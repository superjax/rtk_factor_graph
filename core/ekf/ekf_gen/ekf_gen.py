from core.ekf.ekf_gen.ekf_gen_test import gen_test
import os
import re
from numpy.lib.arraysetops import isin
import yaml

from argparse import ArgumentParser
from scipy.stats import chi2
import numpy as np

from core.ekf.ekf_gen.ekf_gen_utils import (
    accumulate_expr, make_camel, make_snake, State, includes, begin_namespaces, end_namespaces,
    get_matching_elements, jac_member_name
)
from core.ekf.ekf_gen.ekf_gen_templates import DO_UPDATE_TEMPLATE, EKF_HEADER_TEMPLATE, EKF_IMPL_TEMPLATE, TYPE_HEADER_TEMPLATE, TYPE_IMPL_TEMPLATE
from core.ekf.ekf_gen.ekf_gen_matrix_manip import block_sparse_propagate_covariance, sparse_meas_update
from core.ekf.ekf_gen.ekf_gen_state import state_header, state_impl


def pad(text):
    return "\n".join("    " + line for line in text.split('\n'))


def make_jacobian(name, matrix, row_states, col_states):
    JACOBIAN_TEMPLATE = """class {Name} {{
 public:
    {MEMBERS}
    {BUFFERS}
    static {Name} Random();
    static {Name} Zero();
    void setZero();
    void setRandom();
    {DENSE_TYPE} dense({DENSE_ARGS}) const;
}};"""

    def buffer_name(s1, s2):
        return f"{jac_member_name(s1, s2)}_buf_"

    def make_buffers(matrix):
        if not any(any(sr.variable or sc.variable for sr, sc in row) for row in matrix):
            return ""

        out = []
        for row in matrix:
            for sr, sc in row:
                if sr.variable or sc.variable:
                    out.append(f"Mat<{sr.dx}, {sc.dx}> {buffer_name(sr, sc)};")
        return "\n    ".join(out)

    def dim(s):
        if s.variable:
            return f"{s.base_dx} * {s.counter_var}"
        else:
            return f"{s.dx}"

    def dense_type():
        if [s.variable for s in row_states + col_states]:
            return "Eigen::MatrixXd"
        else:
            num_rows = sum([s.dx for s in row_states])
            num_cols = sum([s.dx for s in col_states])
            return f"Mat<{num_rows}, {num_cols}>"

    def dense_args():
        args = {f"int {s.counter_var}" for s in row_states + col_states if s.variable}
        return ", ".join(args)

    def make_getters(matrix):
        out = []
        for row in matrix:
            for sr, sc in row:
                if not sr.variable and not sc.variable:
                    out.append(f"Mat<{sr.dx}, {sc.dx}> {jac_member_name(sr, sc)};")
                if sr.variable or sc.variable:
                    args = {f"int {s.counter_var}" for s in [sr, sc] if s.variable}
                    block = f"{buffer_name(sr, sc)}.block(0, 0, {dim(sr)}, {dim(sc)})"
                    out.append(
                        f"inline auto {jac_member_name(sr, sc)}({', '.join(args)}){{ return {block}; }}"
                    )
                    out.append(
                        f"inline auto {jac_member_name(sr, sc)}({', '.join(args)}) const {{ return {block}; }}"
                    )
        return "\n    ".join(out)

    return JACOBIAN_TEMPLATE.format(
        Name=name,
        BUFFERS=make_buffers(matrix),
        MEMBERS=make_getters(matrix),
        DENSE_TYPE=dense_type(),
        DENSE_ARGS=dense_args()
    )


def make_jacobian_is_finite(matrix, name):
    out = [f"bool isFinite(const {name}& x) {{"]
    out.append("    bool is_finite = true;")
    for row in matrix:
        for sr, sc in row:
            if not sr.variable and not sc.variable:
                out.append(f"    is_finite &= isFinite(x.{jac_member_name(sr, sc)});")
            else:
                out.append(f"    is_finite &= isFinite(x.{jac_member_name(sr, sc)}_buf_);")
    out.append("    return is_finite;")
    out.append("}")
    return "\n".join(out)


def make_jacobian_random_impl(matrix, name):
    out = [f"{name} {name}::Random() {{"]
    out.append(f"    {name} out;")
    out.append("    out.setRandom();")
    out.append("    return out;")
    out.append("}")
    out.append("")
    out.append(f"void {name}::setRandom() {{")
    for row in matrix:
        for sr, sc in row:
            if not sr.variable and not sc.variable:
                out.append(f"    {jac_member_name(sr, sc)}.setRandom();")
            else:
                out.append(f"    {jac_member_name(sr, sc)}_buf_.setRandom();")
    out.append("}")
    return "\n".join(out)


def make_jacobian_zero_impl(matrix, name):
    out = [f"{name} {name}::Zero() {{"]
    out.append(f"    {name} out;")
    out.append("    out.setZero();")
    out.append("    return out;")
    out.append("}")
    out.append("")
    out.append(f"void {name}::setZero() {{")
    for row in matrix:
        for sr, sc in row:
            if not sr.variable and not sc.variable:
                out.append(f"    {jac_member_name(sr, sc)}.setZero();")
            else:
                out.append(f"    {jac_member_name(sr, sc)}_buf_.setZero();")
    out.append("}")
    return "\n".join(out)


def make_jacobian_dense_impl(matrix, name, row_states, col_states):
    def dense_type():
        if [s.variable for s in row_states + col_states]:
            return "Eigen::MatrixXd"
        else:
            num_rows = sum([s.dx for s in row_states])
            num_cols = sum([s.dx for s in col_states])
            return f"Mat<{num_rows}, {num_cols}>"

    def dense_args():
        args = {f"int {s.counter_var}" for s in row_states + col_states if s.variable}
        return ", ".join(args)

    def dim(s):
        if s.variable:
            return f"({s.counter_var}*{s.base_dx})"
        else:
            return s.dx

    def in_matrix(rs, cs, matrix):
        for row in matrix:
            if (rs, cs) in row:
                return True
        return False

    out = [f"{dense_type()} {name}::dense({dense_args()}) const {{{{"]
    out.append(f"    {dense_type()} out({{row_accum}}, {{col_accum}});")
    row_expr = 0
    for r, rs in enumerate(row_states):
        col_expr = 0
        for c, cs in enumerate(col_states):
            if rs.variable or cs.variable:
                args = {f"{s.counter_var}" for s in (rs, cs) if s.variable}
                assigner = f"out.block({row_expr}, {col_expr}, {dim(rs)}, {dim(cs)})"
                getter = f"{jac_member_name(rs, cs)}({', '.join(args)})"
            else:
                assigner = f"out.block<{dim(rs)}, {dim(cs)}>({row_expr}, {col_expr})"
                getter = f"{jac_member_name(rs, cs)}"

            if in_matrix(rs, cs, matrix):
                out.append(f"    {assigner} = {getter};")
            else:
                out.append(f"    {assigner}.setZero();")
            col_expr = accumulate_expr(col_expr, dim(cs))
        row_expr = accumulate_expr(row_expr, dim(rs))
    out.append("    return out;")
    out.append("}}")
    return "\n".join(out).format(row_accum=row_expr, col_accum=col_expr)


def dynamics_jacobian(cfg):
    states = cfg['state_composition']
    jac = []
    for row_state, list in cfg['dynamics'].items():
        jac_row = []
        for col_state in list[0]:
            sr = get_matching_elements(states, lambda x: x.name == row_state)
            sc = get_matching_elements(states, lambda x: x.name == col_state)
            jac_row.append((sr, sc))
        jac.append(jac_row)
    return make_jacobian("StateJac", jac, states, states)


def make_dynamics_jacobian_matrix(cfg):
    states = cfg['state_composition']
    jac = []
    for row_state, list in cfg['dynamics'].items():
        jac_row = []
        for col_state in list[0]:
            sr = get_matching_elements(states, lambda x: x.name == row_state)
            sc = get_matching_elements(states, lambda x: x.name == col_state)
            jac_row.append((sr, sc))
        jac.append(jac_row)
    return jac


def dynamics_jacobian_helpers(cfg):
    jac = make_dynamics_jacobian_matrix(cfg)
    states = cfg['state_composition']
    random_impl = make_jacobian_random_impl(jac, "StateJac")
    zero_impl = make_jacobian_zero_impl(jac, "StateJac")
    dense_impl = make_jacobian_dense_impl(jac, "StateJac", states, states)

    return random_impl + "\n\n" + zero_impl + "\n\n" + dense_impl


def input_jacobian(cfg):
    states = cfg['state_composition']
    inputs = cfg['input']
    jac = []
    for row_state, list in cfg['dynamics'].items():
        jac_row = []
        for col_state in list[1]:
            sr = get_matching_elements(states, lambda x: x.name == row_state)
            sc = get_matching_elements(inputs, lambda x: x.name == col_state)
            jac_row.append((sr, sc))
        jac.append(jac_row)
    return make_jacobian("InputJac", jac, states, inputs)


def make_input_jacobian_matrix(cfg):
    states = cfg['state_composition']
    inputs = cfg['input']
    jac = []
    for row_state, list in cfg['dynamics'].items():
        jac_row = []
        for col_state in list[1]:
            sr = get_matching_elements(states, lambda x: x.name == row_state)
            sc = get_matching_elements(inputs, lambda x: x.name == col_state)
            jac_row.append((sr, sc))
        jac.append(jac_row)
    return jac


def input_jacobian_helpers(cfg):
    states = cfg['state_composition']
    inputs = cfg['input']
    jac = make_input_jacobian_matrix(cfg)
    random_impl = make_jacobian_random_impl(jac, "InputJac")
    zero_impl = make_jacobian_zero_impl(jac, "InputJac")
    dense_impl = make_jacobian_dense_impl(jac, "InputJac", states, inputs)

    return random_impl + "\n\n" + zero_impl + "\n\n" + dense_impl


def meas_jacobian(cfg, meas_key):
    states = cfg['state_composition']
    meas_cfg = cfg["measurements"][meas_key]

    jac = []
    row = []
    sr = State(None, meas_cfg["type"])
    sr.counter_var = f"num_{meas_key}"
    for col_state in meas_cfg["states"]:
        sc = get_matching_elements(states, lambda x: x.name == col_state)
        row.append((sr, sc))
    jac.append(row)
    jacobian_decl = make_jacobian("Jac", jac, [sr], states)

    return "\n    ".join(jacobian_decl.split("\n"))


def make_meas_jacobian_matrix(cfg, meas_key):
    states = cfg['state_composition']
    meas_cfg = cfg["measurements"][meas_key]

    jac = []
    row = []
    sr = State(None, meas_cfg["type"])
    sr.counter_var = f"num_{meas_key}"
    for col_state in meas_cfg["states"]:
        sc = get_matching_elements(states, lambda x: x.name == col_state)
        row.append((sr, sc))
    jac.append(row)
    return jac


def meas_jacobian_helpers(cfg, meas_key):
    jac = make_meas_jacobian_matrix(cfg, meas_key)
    identifier = f"{make_camel(meas_key)}Meas::Jac"
    meas_cfg = cfg["measurements"][meas_key]
    states = cfg['state_composition']
    sr = State(None, meas_cfg["type"])
    out = make_jacobian_random_impl(jac, identifier)
    out += "\n\n" + make_jacobian_zero_impl(jac, identifier)
    out += "\n\n" + make_jacobian_dense_impl(jac, identifier, [sr], states)
    return out


def meas_types(cfg):
    MEAS_TEMPLATE = """class {Name} {{{USING_STATEMENT}
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static constexpr int MAX_SIZE = {MAX_SIZE};
    using Residual = {RESIDUAL_TYPE};
    using Covariance = {COVARIANCE_TYPE};
    using ZType = {Z_TYPE};

    {CONSTANTS}

    static constexpr double MAX_MAHAL = {MAX_MAHAL};
    static constexpr double MAX_PROB = {MAX_PROB};
    static constexpr bool DISABLED = {DISABLED};

    {JACOBIAN}

    ZType z;
    {METADATA}
}};"""

    def make_meas(cfg, key):
        meas_cfg = cfg["measurements"][key]
        meas_state = State(key, meas_cfg['type'])

        def residual_type():
            return f"Vec<{meas_state.dx}>"

        def cov_type():
            if meas_state.variable:
                return f"Vec<{meas_state.base_dx}>"
            else:
                return f"Vec<{meas_state.dx}>"

        def z_type():
            if meas_state.variable:
                return f"std::vector<{meas_state.type}>"
            else:
                return f"{meas_state.type}"

        def disabled():
            if "disabled" in meas_cfg and meas_cfg["disabled"]:
                return "true"
            else:
                return "false"

        def metadata():
            if 'metadata' not in meas_cfg.keys():
                return "// no metadata"
            out = []
            for kind, name in meas_cfg['metadata']:
                out.append(f"{kind} {name};")
            return "\n        ".join(out)

        def constants():
            if 'constants' not in meas_cfg.keys():
                return "// no constants"
            out = []
            for kind, name, value in meas_cfg['constants']:
                out.append(f"static constexpr {kind} {name} = {value};")
            return "\n        ".join(out)

        def using_statement():
            if meas_state.manifold:
                return f"using {meas_state.type} = math::{meas_state.type}<double>;"
            else:
                return ""

        name = f"{make_camel(key)}Meas"

        return MEAS_TEMPLATE.format(
            Name=name,
            USING_STATEMENT=using_statement(),
            RESIDUAL_TYPE=residual_type(),
            COVARIANCE_TYPE=cov_type(),
            Z_TYPE=z_type(),
            MAX_MAHAL=compute_max_mahal(cfg, key),
            MAX_PROB=meas_cfg['gating_probability'],
            MAX_SIZE=meas_state.dx,
            DISABLED=disabled(),
            METADATA=metadata(),
            JACOBIAN=meas_jacobian(cfg, key),
            CONSTANTS=constants()
        )

    meas_types = []
    for key in cfg["measurements"].keys():
        meas_types.append(make_meas(cfg, key))
    return "\n\n".join(meas_types)


def meas_is_finite(cfg, key):
    namespaces = "::".join(cfg["namespaces"])
    meas_cfg = cfg["measurements"][key]
    meas_state = State(key, meas_cfg['type'])
    if meas_state.variable:
        out = f"bool isFinite(const {namespaces}::{make_camel(key)}Meas::ZType& z) {{\n"
        out += "    bool is_finite = true;\n"
        out += "    for (size_t i = 0; i < z.size(); ++i) {\n"
        out += "        is_finite &= isFinite(z[i]);\n"
        out += "    }\n"
        out += "    return is_finite;\n"
        out += "}"
        return out
    return ""


def meas_types_helpers(cfg):
    out = []
    for key in cfg["measurements"].keys():
        out.append(meas_jacobian_helpers(cfg, key))
    return "\n\n".join(out)


def list_meas_types(cfg):
    out = []
    for k, item in cfg["measurements"].items():
        out.append(f"typename {cfg['kalman_filter_class']}::{make_camel(k)}Meas")
    return ", ".join(out)


def compute_max_mahal(cfg, key):
    meas_cfg = cfg["measurements"][key]
    max_prob = meas_cfg["gating_probability"]
    meas_state = State(key, meas_cfg["type"])
    if max_prob < 1.0:
        meas_size = meas_state.dx
        x = np.arange(0, 1000, 0.01)
        chi2_cdf = chi2.cdf(x, meas_size - 1)
        idx = (np.abs(chi2_cdf - max_prob)).argmin()
        return f"{x[idx]}"
    else:
        return "std::numeric_limits<double>::max()"


def get_variable_states(cfg):
    states = cfg['state_composition']
    out = [""]
    for s in states:
        if not s.variable:
            continue
        size_var = re.sub("num_", "size_", s.counter_var)
        out.append(f"const auto& {s.counter_var} = x.{s.counter_var};")
        out.append(f"const int {size_var} = {s.base_dx} * x.{s.counter_var};")
    return "\n    ".join(out)


def get_variable_meas(cfg, key):
    meas_cfg = cfg["measurements"][key]
    meas_state = State(key, meas_cfg['type'])
    if meas_state.variable:
        out = []
        size_var = re.sub("num_", "size_", meas_state.counter_var)
        out.append(f"const int {size_var} = {meas_state.base_dx} * z.size();")
        out.append(f"const auto& {meas_state.counter_var} = z.size();")
        return "\n    ".join(out)
    else:
        return ""


def bail_if_variable_meas_empty(cfg, key):
    meas_cfg = cfg["measurements"][key]
    meas_state = State(key, meas_cfg['type'])

    if meas_state.variable:
        out = []
        out.append(f"if ({meas_state.counter_var} == 0) {{")
        out.append("    return Error::create(\"Empty Measurement\");")
        out.append("}")
        return "\n    ".join(out)
    else:
        return ""


def is_finite_decls(cfg):
    namespaces = "::".join(cfg['namespaces'])
    out = []
    out.append(f"bool isFinite(const {namespaces}::StateJac& jac);")
    out.append(f"bool isFinite(const {namespaces}::InputJac& jac);")
    for key in cfg["measurements"]:
        meas_cfg = cfg["measurements"][key]
        meas_state = State(key, meas_cfg['type'])
        type_name = f"{make_camel(key)}Meas"
        if meas_state.variable:
            out.append(f"bool isFinite(const {namespaces}::{type_name}::ZType& z);")
        out.append(f"bool isFinite(const {namespaces}::{type_name}::Jac& z);")
    return "\n".join(out)


def is_finite_impls(cfg):
    namespaces = "::".join(cfg['namespaces'])
    out = []
    out.append(
        make_jacobian_is_finite(make_dynamics_jacobian_matrix(cfg), f"{namespaces}::StateJac")
    )
    out.append(make_jacobian_is_finite(make_input_jacobian_matrix(cfg), f"{namespaces}::InputJac"))
    for key in cfg["measurements"]:
        identifier = f"{make_camel(key)}Meas::Jac"
        out.append(
            make_jacobian_is_finite(
                make_meas_jacobian_matrix(cfg, key), f"{namespaces}::{identifier}"
            )
        )
        out.append(meas_is_finite(cfg, key))
    return "\n".join(out)


def types_header(cfg):
    return TYPE_HEADER_TEMPLATE.format(
        DESTINATION_DIR=cfg["destination"],
        EKF_FILENAME=cfg["ekf_filename_base"],
        BEGIN_NAMESPACES=begin_namespaces(cfg),
        DYNAMICS_JACOBIAN=dynamics_jacobian(cfg),
        INPUT_JACOBIAN=input_jacobian(cfg),
        MEAS_TYPES=meas_types(cfg),
        IS_FINITE_DECLS=is_finite_decls(cfg),
        END_NAMESPACES=end_namespaces(cfg)
    )


def types_impl(cfg):
    return TYPE_IMPL_TEMPLATE.format(
        DESTINATION_DIR=cfg["destination"],
        EKF_FILENAME=cfg["ekf_filename_base"],
        BEGIN_NAMESPACES=begin_namespaces(cfg),
        DYNAMICS_JACOBIAN_HELPERS=dynamics_jacobian_helpers(cfg),
        INPUT_JACOBIAN_HELPERS=input_jacobian_helpers(cfg),
        MEAS_TYPES_HELPERS=meas_types_helpers(cfg),
        IS_FINITE_IMPLS=is_finite_impls(cfg),
        END_NAMESPACES=end_namespaces(cfg),
    )


def ekf_header(cfg):
    return EKF_HEADER_TEMPLATE.format(
        DESTINATION_DIR=cfg["destination"],
        EKF_FILENAME=cfg["ekf_filename_base"],
        INCLUDES=includes(cfg),
        BEGIN_NAMESPACES=begin_namespaces(cfg),
        EkfType=cfg["kalman_filter_class"],
        ErrorState=cfg["error_state_name"],
        State=cfg["state_name"],
        Input=cfg["input_name"],
        END_NAMESPACES=end_namespaces(cfg),
        GET_VARIABLE_STATES=get_variable_states(cfg)
    )


def ekf_impl(cfg):
    def do_updates():
        out = []
        for key in cfg["measurements"].keys():
            out.append(
                DO_UPDATE_TEMPLATE.format(
                    **sparse_meas_update(cfg, key),
                    Meas=f"{make_camel(key)}Meas",
                    ErrorState=cfg["error_state_name"],
                    GET_VARIABLE_STATES=get_variable_states(cfg),
                    GET_VARIABLE_MEAS=get_variable_meas(cfg, key),
                    BAIL_IF_VARIABLE_MEAS_EMPTY=bail_if_variable_meas_empty(cfg, key)
                )
            )
        return "\n".join(out)

    return EKF_IMPL_TEMPLATE.format(
        DESTINATION_DIR=cfg["destination"],
        ErrorState=cfg["error_state_name"],
        Input=cfg["input_name"],
        EKF_FILENAME=make_snake(cfg["kalman_filter_class"]),
        BEGIN_NAMESPACES=begin_namespaces(cfg),
        DO_UPDATES=do_updates(),
        GET_VARIABLE_STATES=get_variable_states(cfg),
        **block_sparse_propagate_covariance(cfg),
        END_NAMESPACES=end_namespaces(cfg),
    )


def meas_to_state(cfg, meas_key):
    return State(meas_key, cfg["measurements"][meas_key]["type"])


def organize_states(state_dictionary):
    """ Move variable states to the end, and compute error state size, state size """
    states = []
    for k, v in state_dictionary.items():
        states.append(State(name=k, kind=v))
    return [s for s in states if not s.variable] + [s for s in states if s.variable]


if __name__ == "__main__":
    parser = ArgumentParser("Generate an EKF")
    parser.add_argument(
        "--config_file", help="configuration file to generate EKF from", required=True
    )
    parser.add_argument("--destination", help="destination directoy of c++ code", required=True)
    args = parser.parse_args()

    cfg = yaml.load(open(args.config_file).read(), Loader=yaml.FullLoader)
    cfg["destination"] = args.destination

    cfg["kalman_filter_class"] = make_camel(
        os.path.splitext(os.path.basename(args.config_file))[0], True
    )

    cfg["state_composition"] = organize_states(cfg["state_composition"])
    cfg["input"] = organize_states(cfg["input"])

    if not os.path.exists(cfg["destination"]):
        os.mkdir(cfg["destination"])

    ekf_filename_base = make_snake(cfg["kalman_filter_class"])
    cfg["ekf_filename_base"] = ekf_filename_base
    with open(os.path.join(cfg["destination"], ekf_filename_base + "_state.h"), 'w') as f:
        f.write(state_header(cfg))

    with open(os.path.join(cfg["destination"], ekf_filename_base + "_state.cxx"), 'w') as f:
        f.write(state_impl(cfg))

    with open(os.path.join(cfg["destination"], ekf_filename_base + "_types.h"), 'w') as f:
        f.write(types_header(cfg))

    with open(os.path.join(cfg["destination"], ekf_filename_base + "_types.cxx"), 'w') as f:
        f.write(types_impl(cfg))

    with open(os.path.join(cfg["destination"], ekf_filename_base + ".h"), 'w') as f:
        f.write(ekf_header(cfg))

    with open(os.path.join(cfg["destination"], ekf_filename_base + ".cxx"), 'w') as f:
        f.write(ekf_impl(cfg))

    with open(os.path.join(cfg["destination"], ekf_filename_base + "_sparse_test.cxx"), 'w') as f:
        f.write(gen_test(cfg))
