import os
import yaml
from itertools import accumulate

from argparse import ArgumentParser
from scipy.stats import chi2
import numpy as np

from core.ekf.ekf_gen.ekf_gen_utils import make_camel, make_snake, is_lie, get_size
from core.ekf.ekf_gen.ekf_gen_templates import (
    STATE_HEADER_TEMPLATE, STATE_IMPL_TEMPLATE, EKF_HEADER_TEMPLATE
)
from core.ekf.ekf_gen.ekf_gen_matrix_manip import block_sparse_propagate_covariance, jacobian_name


def using_statements(types):
    needed_types = {t for t in types if not t.startswith("Vec")}
    return "\n    ".join(f"using {t} = math::{t}<double>;" for t in needed_types)


def begin_namespaces(cfg):
    return "\n".join([f"namespace {ns} {{" for ns in cfg['namespaces']])


def includes(cfg):
    if 'includes' not in cfg.keys():
        return ""
    else:
        return "\n".join([f'#include "{inc}' for inc in cfg['includes']])


def end_namespaces(cfg):
    return "\n".join(f"}} // end namespace {ns}" for ns in reversed(cfg['namespaces']))


def input_size(cfg):
    sizes = [get_size(v)[0] for _, v in cfg["input"].items()]
    return list(accumulate(sizes))[-1]


def dof(cfg):
    sizes = [get_size(v)[1] for _, v in cfg["state_composition"].items()]
    return list(accumulate(sizes))[-1]


def error_state_indexes(cfg):
    keys = [k for k, v in cfg["state_composition"].items()]
    sizes = [get_size(v)[1] for k, v in cfg["state_composition"].items()]
    indexes = list(accumulate(sizes))
    indexes.insert(0, 0)
    keys.append('SIZE')
    return ",\n        ".join(f"{k.upper()} = {s}" for k, s in zip(keys, indexes))


def state_indexes(cfg):
    keys = [k for k, v in cfg["state_composition"].items()]
    sizes = [get_size(v)[0] for k, v in cfg["state_composition"].items()]
    indexes = list(accumulate(sizes))
    indexes.insert(0, 0)
    keys.append('SIZE')
    return ",\n        ".join(f"{k.upper()} = {s}" for k, s in zip(keys, indexes))


def input_indexes(cfg):
    keys = [k for k, v in cfg["input"].items()]
    sizes = [get_size(v)[0] for k, v in cfg["input"].items()]
    indexes = list(accumulate(sizes))
    indexes.insert(0, 0)
    keys.append('SIZE')
    return ",\n        ".join(f"{k.upper()} = {s}" for k, s in zip(keys, indexes))


def error_state_accessors(cfg):
    size_key = ((get_size(v)[1], k) for k, v in cfg["state_composition"].items())
    return "\n    ".join(f"Eigen::Map<Vec{size}> {key};" for size, key in size_key)


def state_accessors(cfg):
    def get_type(state):
        if state.startswith("Vec"):
            return f"Eigen::Map<{state}>"
        else:
            return state

    kind_key = ((get_type(v), k) for k, v in cfg["state_composition"].items())
    return "\n    ".join(f"{kind} {key};" for kind, key in kind_key)


def input_accessors(cfg):
    def get_type(kind):
        if kind.startswith("Vec"):
            return f"Eigen::Map<{kind}>"
        else:
            return kind

    kind_key = ((get_type(v), k) for k, v in cfg["input"].items())
    return "\n    ".join(f"{kind} {key};" for kind, key in kind_key)


def extra_typedefs(cfg):
    return ""


def error_state_init_list(cfg):
    return ",\n    ".join(
        f"{key}(this->data() + {key.upper()})" for key in cfg["state_composition"].keys()
    )


def state_init_list(cfg):
    return ",\n    ".join(
        f"{key}(arr.data() + {key.upper()})" for key in cfg["state_composition"].keys()
    )


def input_init_list(cfg):
    return ",\n    ".join(f"{key}(this->data() + {key.upper()})" for key in cfg["input"].keys())


def boxplus_impl(cfg):
    lines = ["State xp;"]
    for k, v in cfg["state_composition"].items():
        if is_lie(v):
            lines.append(f"xp.{k} = {v}::exp(dx.{k}) * {k};")
        else:
            lines.append(f"xp.{k} = dx.{k} + {k};")
    lines.append("return xp;")
    return "\n    ".join(lines)


def boxplus_vector_impl(cfg):
    lines = ["State xp;"]
    ErrorState = cfg["error_state_name"]
    for k, v in cfg["state_composition"].items():
        _, size = get_size(v)
        dx = f"dx.segment<{size}>({ErrorState}::{k.upper()})"
        if is_lie(v):
            lines.append(f"xp.{k} =  {k} * {v}::exp({dx});")
        else:
            lines.append(f"xp.{k} = {dx} + {k};")
    lines.append("return xp;")
    return "\n    ".join(lines)


def self_plus_impl(cfg):
    lines = []
    for k, v in cfg["state_composition"].items():
        if is_lie(v):
            lines.append(f"{k} = {k}* {v}::exp(dx.{k});")
        else:
            lines.append(f"{k} += dx.{k};")
    lines.append("return *this;")
    return "\n    ".join(lines)


def self_plus_vector_impl(cfg):
    lines = []
    ErrorState = cfg["error_state_name"]
    for k, v in cfg["state_composition"].items():
        _, size = get_size(v)
        dx = f"dx.segment<{size}>({ErrorState}::{k.upper()})"
        if is_lie(v):
            lines.append(f"{k} = {k} * {v}::exp({dx});")
        else:
            lines.append(f"{k} += {dx};")
    lines.append("return *this;")
    return "\n    ".join(lines)


def boxminus_impl(cfg):
    lines = [f"{cfg['error_state_name']} dx;"]
    for k, v in cfg["state_composition"].items():
        if is_lie(v):
            lines.append(f"dx.{k} = (x2.{k}.inverse() * {k}).log();")
        else:
            lines.append(f"dx.{k} = {k} - x2.{k};")
    lines.append("return dx;")
    return "\n    ".join(lines)


def state_random_impl(cfg):
    lines = [f"{cfg['state_name']} x;"]
    for k, v in cfg["state_composition"].items():
        lines.append(f"x.{k} = {v}::Random();")
    lines.append("return x;")
    return "\n    ".join(lines)


def identity_impl(cfg):
    lines = [f"{cfg['state_name']} x;"]
    for k, v in cfg["state_composition"].items():
        if is_lie(v):
            lines.append(f"x.{k} = {v}::Identity();")
        else:
            lines.append(f"x.{k}.setZero();")
    lines.append("return x;")
    return "\n    ".join(lines)


def jac_types(cfg):
    def get_block(rows, cols, rs, cs, key1, key2):
        return f"return this->template block<{rows}, {cols}>({key1}::{rs.upper()}, {key2}::{cs.upper()});"    # noqa

    state_jac = "Mat<ErrorState::SIZE, ErrorState::SIZE>"
    lines = [f"struct StateJac : public {state_jac} {{"]
    lines.append("    StateJac() { setZero(); }")
    for k_state, bys in cfg["dynamics"].items():
        k = cfg["state_composition"][k_state]
        for v_name in bys[0]:
            v = cfg["state_composition"][v_name]
            rows = get_size(k)[1]
            cols = get_size(v)[1]
            jac_name = jacobian_name(k_state, v_name)
            block_type = f"BlkMat<{state_jac}, {rows}, {cols}>"
            const_block_type = f"const BlkMat<const {state_jac}, {rows}, {cols}> "
            impl = get_block(rows, cols, k_state, v_name, "ErrorState", "ErrorState")
            lines.append(f"    {block_type} {jac_name}() {{ {impl} }}")
            lines.append(f"    {const_block_type} {jac_name}() const {{ {impl} }}")
    lines.append("};\n\n")

    in_size = input_size(cfg)
    input_jac = f"Mat<ErrorState::SIZE, {in_size}>"
    lines.append(f"struct InputJac : public {input_jac} {{")
    lines.append("    InputJac() { setZero(); }")
    for k_state, bys in cfg["dynamics"].items():
        k = cfg["state_composition"][k_state]
        for v_name in bys[1]:
            v = cfg["input"][v_name]
            rows = get_size(k)[1]
            cols = get_size(v)[1]
            jac_name = f"d{make_camel(k_state).title()}d{make_camel(v_name).title()}"
            block_type = f"BlkMat<{input_jac}, {rows}, {cols}>"
            const_block_type = f"const BlkMat<const {input_jac}, {rows}, {cols}> "
            impl = get_block(rows, cols, k_state, v_name, "ErrorState", "Input")
            lines.append(f"    {block_type} {jac_name}() {{ {impl} }}")
            lines.append(f"    {const_block_type} {jac_name}() const {{ {impl} }}")
    lines.append("};\n\n")
    return "\n    ".join(lines)


def meas_types(cfg):
    def make_jacobian(meas_cfg):
        out_size = meas_size(meas_cfg)
        lines = [f"    struct Jac : public Mat<{out_size}, ErrorState::SIZE> {{"]
        lines.append("    Jac() { setZero(); }")

        def grab_block(cols, s):
            return f"{{ return this->template block<{out_size}, {cols}>(0, ErrorState::{s.upper()}); }}"    # noqa

        def make_type(ss, const=False):
            const = "const " if const else ""
            return f"BlkMat<{const}Mat<{out_size}, ErrorState::SIZE>, {out_size}, {ss}>"

        for s in meas_cfg["states"]:
            ss = get_size(cfg["state_composition"][s])[1]
            lines.append(f"    {make_type(ss)} d{s.capitalize()}() {grab_block(ss, s)}")
            lines.append(
                f"    const {make_type(ss, True)} d{s.capitalize()}() const {grab_block(ss, s)}"
            )
        lines.append("};")
        return "\n        ".join(lines)

    def make_meas(cfg, key):
        meas_cfg = cfg["measurements"][key]
        lines = []
        lines.append(f"struct {make_camel(key)}Meas {{")
        lines.append(f"    using Residual = Vec<{meas_size(meas_cfg)}>;")
        lines.append(f"    using Covariance = DiagMat<{meas_size(meas_cfg)}>;")
        lines.append(f"    using ZType = {meas_cfg['type']};")
        lines.append(f"    static constexpr double MAX_MAHAL = {compute_max_mahal(cfg, key)};")
        lines.append(f"    static constexpr double MAX_PROB = {meas_cfg['gating_probability']};")
        lines.append(f"    static constexpr bool DISABLED = {disabled(cfg, key)};")
        lines.append(f"    static constexpr int SIZE = {meas_size(meas_cfg)};")
        lines.append("    ZType z;")
        if 'metadata' in meas_cfg.keys():
            for item in meas_cfg['metadata']:
                lines.append(f"    {item[0]} {item[1]};")
        if 'constants' in meas_cfg.keys():
            for item in meas_cfg['constants']:
                lines.append(f"    static constexpr {item[0]} {item[1]} = {item[2]};")
        lines.append("")
        lines.append(make_jacobian(meas_cfg))
        lines.append("};")
        return "\n    ".join(lines)

    meas_types = []
    for key in cfg["measurements"].keys():
        meas_types.append(make_meas(cfg, key))

    return "\n\n    ".join(meas_types)


def list_meas_types(cfg):
    out = []
    for k, item in cfg["measurements"].items():
        out.append(f"typename {cfg['kalman_filter_class']}::{make_camel(k)}Meas")
    return ", ".join(out)


def compute_max_mahal(cfg, key):
    meas_cfg = cfg["measurements"][key]
    max_prob = meas_cfg["gating_probability"]
    if max_prob < 1.0:
        meas_size = meas_cfg["size"]
        x = np.arange(0, 1000, 0.01)
        chi2_cdf = chi2.cdf(x, meas_size - 1)
        idx = (np.abs(chi2_cdf - max_prob)).argmin()
        return f"{x[idx]}"
    else:
        return "std::numeric_limits<double>::max()"


def disabled(cfg, key):
    meas_cfg = cfg["measurements"][key]
    if "disabled" in meas_cfg and meas_cfg["disabled"]:
        return "true"
    else:
        return "false"


def meas_size(meas_cfg):
    if "size" in meas_cfg:
        return meas_cfg["size"]
    else:
        return get_size(meas_cfg["type"])[1]


def state_header(cfg):
    return STATE_HEADER_TEMPLATE.format(
        BEGIN_NAMESPACES=begin_namespaces(cfg),
        USING_STATEMENTS=using_statements([v for _, v in cfg["state_composition"].items()]),
        ErrorState=cfg["error_state_name"],
        State=cfg["state_name"],
        ERROR_STATE_INDEXES=error_state_indexes(cfg),
        ERROR_STATE_ACCESSORS=error_state_accessors(cfg),
        STATE_INDEXES=state_indexes(cfg),
        STATE_ACCESSORS=state_accessors(cfg),
        Input=cfg["input_name"],
        INPUT_ACCESSORS=input_accessors(cfg),
        INPUT_INDEXES=input_indexes(cfg),
        EXTRA_TYPEDEFS=extra_typedefs(cfg),
        INPUT_SIZE=input_size(cfg),
        DOF=dof(cfg),
        END_NAMESPACES=end_namespaces(cfg)
    )


def state_impl(cfg):
    return STATE_IMPL_TEMPLATE.format(
        DESTINATION_DIR=cfg["destination"],
        BEGIN_NAMESPACES=begin_namespaces(cfg),
        ErrorState=cfg["error_state_name"],
        State=cfg["state_name"],
        ERROR_STATE_INIT_LIST=error_state_init_list(cfg),
        STATE_INIT_LIST=state_init_list(cfg),
        BOXPLUS_IMPL=boxplus_impl(cfg),
        BOXPLUS_VECTOR_IMPL=boxplus_vector_impl(cfg),
        SELF_PLUS_IMPL=self_plus_impl(cfg),
        SELF_PLUS_VECTOR_IMPL=self_plus_vector_impl(cfg),
        Input=cfg["input_name"],
        INPUT_INIT_LIST=input_init_list(cfg),
        BOXMINUS_IMPL=boxminus_impl(cfg),
        STATE_RANDOM_IMPL=state_random_impl(cfg),
        IDENTITY_IMPL=identity_impl(cfg),
        DOF=dof(cfg),
        END_NAMESPACES=end_namespaces(cfg)
    )


def ekf_header(cfg):
    return EKF_HEADER_TEMPLATE.format(
        DESTINATION_DIR=cfg["destination"],
        INCLUDES=includes(cfg),
        BEGIN_NAMESPACES=begin_namespaces(cfg),
        EkfType=cfg["kalman_filter_class"],
        ErrorState=cfg["error_state_name"],
        USING_STATEMENTS=using_statements([m["type"] for _, m in cfg["measurements"].items()]),
        State=cfg["state_name"],
        MEAS_TYPES=meas_types(cfg),
        JAC_TYPES=jac_types(cfg),
        Input=cfg["input_name"],
        InputSize=input_size(cfg),
        END_NAMESPACES=end_namespaces(cfg),
        PROPAGATE_COVARIANCE=block_sparse_propagate_covariance(cfg),
    )


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

    if not os.path.exists(cfg["destination"]):
        os.mkdir(cfg["destination"])

    with open(os.path.join(cfg["destination"], "state.h"), 'w') as f:
        f.write(state_header(cfg))

    with open(os.path.join(cfg["destination"], "state.cxx"), 'w') as f:
        f.write(state_impl(cfg))

    ekf_filename_base = make_snake(cfg["kalman_filter_class"])
    with open(os.path.join(cfg["destination"], ekf_filename_base + ".h"), 'w') as f:
        f.write(ekf_header(cfg))
