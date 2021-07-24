from itertools import accumulate

from core.ekf.ekf_gen.ekf_gen_templates import STATE_HEADER_TEMPLATE, STATE_IMPL_TEMPLATE
from core.ekf.ekf_gen.ekf_gen_utils import (
    accumulate_expr, make_camel, begin_namespaces, end_namespaces
)


def state_using_statements(cfg):
    states = cfg['state_composition']
    needed_types = {s.type for s in states if s.manifold}
    return "\n    ".join(f"using {t} = math::{t}<double>;" for t in needed_types)


def input_using_statements(cfg):
    states = cfg['input']
    needed_types = {s.type for s in states if s.manifold}
    return "\n    ".join(f"using {t} = math::{t}<double>;" for t in needed_types)


def error_state_buffers(cfg):
    states = cfg['state_composition']
    variable_states = [s for s in states if s.variable]

    out = []
    for s in variable_states:
        out.append(f"Vec<{s.dx}> {s.name}_buf_;")

    return "\n    ".join(out)


def error_state_vector_getters(cfg):
    states = cfg['state_composition']
    variable_states = [s for s in states if s.variable]

    out = []
    for s in variable_states:
        impl = f"return {s.name}_buf_.head({s.base_dx} * {s.counter_var});"
        out.append(f"inline auto {s.name}_vec(int {s.counter_var}) {{ {impl} }}")
        out.append(f"inline const auto {s.name}_vec(int {s.counter_var}) const {{ {impl} }}")

    return "\n    ".join(out)


def error_state_dense_decl(cfg):
    states = cfg['state_composition']
    variable_states = [s for s in states if s.variable]

    if len(variable_states) > 0:
        out_type = "Eigen::VectorXd"
        args = {f"int {s.counter_var}" for s in variable_states}
    else:
        args = {}
        out_type = f"Vec<{sum([s.dx for s in states])}>"
    return f"{out_type} dense({', '.join(args)}) const;"


def error_state_members(cfg):
    states = cfg['state_composition']
    fixed_states = [s for s in states if not s.variable]
    variable_states = [s for s in states if s.variable]

    out = []
    for s in fixed_states:
        out.append(f"Vec<{s.dx}> {s.name};")

    for s in variable_states:
        impl = f"return {s.name}_buf_.segment<{s.base_dx}>(idx * {s.base_dx});"
        out.append(f"inline auto {s.name}(int idx) {{ {impl} }}")
        out.append(f"inline const auto {s.name}(int idx) const {{ {impl} }}")

    return "\n    ".join(out)


def state_members(cfg):
    states = cfg['state_composition']
    fixed_states = [s for s in states if not s.variable]
    variable_states = [s for s in states if s.variable]

    out = []
    for s in fixed_states:
        out.append(f"{s.type} {s.name};")

    if len(variable_states) > 0:
        out.append("")
        for s in variable_states:
            out.append(f"std::array<{s.type}, {s.bundle_size}> {s.name};")

        out.append("")
        for s in variable_states:
            out.append(f"size_t {s.counter_var} = 0;")
            out.append(f"static constexpr int {s.MAX} = {s.bundle_size};")

    return "\n    ".join(out)


def state_size(cfg):
    states = cfg['state_composition']
    accum = 0
    for s in states:
        dim = s.dx if not s.variable else s.counter_var
        accum = accumulate_expr(accum, dim)
    return accum


def dof(states):
    return sum([s.dx for s in states])


def covariance_buffers(cfg):
    states = cfg['state_composition']

    def member_name(s1, s2):
        return f"{make_camel(s1.name)}_{make_camel(s2.name)}"

    def buffer_name(s1, s2):
        return f"{member_name(s1, s2)}_buf_"

    out = []
    for r, s1 in enumerate(states):
        for c, s2 in enumerate(states):
            if r > c:
                continue
            if s1.variable or s2.variable:
                out.append(f"Mat<{s1.dx}, {s2.dx}> {buffer_name(s1, s2)};")
    return "\n    ".join(out)


def covariance_members(cfg):
    states = cfg['state_composition']

    def member_name(s1, s2):
        return f"{make_camel(s1.name)}_{make_camel(s2.name)}"

    def buffer_name(s1, s2):
        return f"{member_name(s1, s2)}_buf_"

    def dim(s):
        if s.variable:
            return f"{s.base_dx} * {s.counter_var}"
        else:
            return f"{s.dx}"

    out = []
    for r, s1 in enumerate(states):
        for c, s2 in enumerate(states):
            if r > c:
                continue
            if not s1.variable and not s2.variable:
                out.append(f"Mat<{s1.dx}, {s2.dx}> {member_name(s1, s2)};")
            if s1.variable or s2.variable:
                args = {f"int {s.counter_var}" for s in [s1, s2] if s.variable}
                block = f"{buffer_name(s1, s2)}.block(0, 0, {dim(s1)}, {dim(s2)})"
                out.append(
                    f"inline auto {member_name(s1, s2)}({', '.join(args)}){{ return {block}; }}"
                )
                out.append(
                    f"inline auto {member_name(s1, s2)}({', '.join(args)}) const {{ return {block}; }}"
                )
    return "\n    ".join(out)


def covariance_set_random(cfg):
    states = cfg['state_composition']

    def member_name(s1, s2):
        if s1.variable or s2.variable:
            return f"{make_camel(s1.name)}_{make_camel(s2.name)}_buf_"
        else:
            return f"{make_camel(s1.name)}_{make_camel(s2.name)}"

    out = []
    for r, s1 in enumerate(states):
        for c, s2 in enumerate(states):
            if c >= r:
                out.append(f"{member_name(s1, s2)}.setRandom();")
            if c == r:
                out.append(
                    f"{member_name(s1, s2)}.triangularView<Eigen::Lower>() = {member_name(s1, s2)}.transpose();"
                )
    return "\n    ".join(out)


def covariance_identity(cfg):
    states = cfg['state_composition']

    def member_name(s1, s2):
        if s1.variable or s2.variable:
            return f"{make_camel(s1.name)}_{make_camel(s2.name)}_buf_"
        else:
            return f"{make_camel(s1.name)}_{make_camel(s2.name)}"

    out = []
    for r, s1 in enumerate(states):
        for c, s2 in enumerate(states):
            if c > r:
                out.append(f"{member_name(s1, s2)}.setZero();")
            if r == c:
                out.append(f"{member_name(s1, s2)}.setIdentity();")
    return "\n    ".join(out)


def covariance_dense_decl(cfg):
    states = cfg['state_composition']
    variable_states = [s for s in states if s.variable]

    if len(variable_states) > 0:
        out_type = "Eigen::MatrixXd"
        args = {f"int {s.counter_var}" for s in variable_states}
    else:
        args = {}
        dx = sum([s.dx for s in states])
        out_type = f"Mat<{dx},{dx}>"
    return f"{out_type} dense({', '.join(args)}) const;"


def copy_state(cfg):
    states = cfg['state_composition']

    out = []
    for s in [s for s in states if not s.variable]:
        out.append(f"{s.name} = other.{s.name};")
    out.append("")
    for s in [s for s in states if s.variable]:
        out.append(f"{s.counter_var} = other.{s.counter_var};")
        out.append(f"for (size_t i = 0; i < other.{s.counter_var}; ++i)")
        out.append("{")
        out.append(f"    {s.name}[i] = other.{s.name}[i];")
        out.append("}")
    return "\n    ".join(out)


def scalar_multiply(cfg):
    states = cfg['state_composition']
    fixed_states = [s for s in states if not s.variable]
    variable_states = [s for s in states if s.variable]
    out = []
    for s in fixed_states:
        out.append(f"out.{s.name} = {s.name} * s;")
    for s in variable_states:
        out.append(f"out.{s.name}_buf_ = {s.name}_buf_ * s;")
    return "\n    ".join(out)


def error_state_minus(cfg):
    states = cfg['state_composition']
    fixed_states = [s for s in states if not s.variable]
    variable_states = [s for s in states if s.variable]
    out = []
    for s in fixed_states:
        out.append(f"out.{s.name} = {s.name} - dx.{s.name};")
    for s in variable_states:
        out.append(f"out.{s.name}_buf_ = {s.name}_buf_ - dx.{s.name}_buf_;")
    return "\n    ".join(out)


def impl_plus(states):
    fixed_states = [s for s in states if not s.variable]
    variable_states = [s for s in states if s.variable]
    out = []
    for s in fixed_states:
        out.append(f"out.{s.name} = {s.name} + dx.{s.name};")
    for s in variable_states:
        out.append(f"out.{s.name}_buf_ = {s.name}_buf_ + dx.{s.name}_buf_;")
    return "\n    ".join(out)


def self_add(states):
    fixed_states = [s for s in states if not s.variable]
    variable_states = [s for s in states if s.variable]
    out = []

    for s in fixed_states:
        out.append(f"{s.name} += dx.{s.name};")
    for s in variable_states:
        out.append(f"{s.name}_buf_ += dx.{s.name}_buf_;")
    return "\n    ".join(out)


def set_random(cfg):
    states = cfg['state_composition']
    fixed_states = [s for s in states if not s.variable]
    variable_states = [s for s in states if s.variable]
    out = []
    for s in fixed_states:
        out.append(f"{s.name}.setRandom();")
    for s in variable_states:
        out.append(f"{s.name}_buf_.setRandom();")
    return "\n    ".join(out)


def input_random(cfg):
    inputs = cfg["input"]
    fixed_states = [s for s in inputs if not s.variable]
    variable_states = [s for s in inputs if s.variable]
    out = []
    for s in fixed_states:
        out.append(f"{s.name}.setRandom();")
    for s in variable_states:
        out.append(f"{s.name}_buf_.setRandom();")
    return "\n    ".join(out)


def input_random_normal(cfg):
    inputs = cfg["input"]
    fixed_states = [s for s in inputs if not s.variable]
    variable_states = [s for s in inputs if s.variable]
    out = []
    for s in fixed_states:
        out.append(f"out.{s.name} = randomNormal<Vec<{s.dx}>>();")
    for s in variable_states:
        out.append(f"out.{s.name}_buf_ = randomNormal<Vec<{s.dx}>>();")
    return "\n    ".join(out)


def multiply_constant(cfg):
    inputs = cfg["input"]
    fixed_states = [s for s in inputs if not s.variable]
    variable_states = [s for s in inputs if s.variable]
    out = []
    for s in fixed_states:
        out.append(f"out.{s.name} = {s.name} * s;")
    for s in variable_states:
        out.append(f"out.{s.name}_buf_ = {s.name}_buf_ * s;")
    return "\n    ".join(out)


def set_zero(cfg):
    states = cfg['state_composition']
    fixed_states = [s for s in states if not s.variable]
    variable_states = [s for s in states if s.variable]
    out = []
    for s in fixed_states:
        out.append(f"{s.name}.setZero();")
    for s in variable_states:
        out.append(f"{s.name}_buf_.setZero();")
    return "\n    ".join(out)


def error_state_constant(cfg):
    states = cfg['state_composition']
    fixed_states = [s for s in states if not s.variable]
    variable_states = [s for s in states if s.variable]
    out = []
    for s in fixed_states:
        out.append(f"out.{s.name}.setConstant(s);")
    for s in variable_states:
        out.append(f"out.{s.name}_buf_.setConstant(s);")
    return "\n    ".join(out)


def input_zero(cfg):
    inputs = cfg["input"]
    fixed_states = [s for s in inputs if not s.variable]
    variable_states = [s for s in inputs if s.variable]
    out = []
    for s in fixed_states:
        out.append(f"{s.name}.setZero();")
    for s in variable_states:
        out.append(f"{s.name}_buf_.setZero();")
    return "\n    ".join(out)


def compute_current_size(cfg):
    states = cfg['state_composition']
    fixed_states = [s for s in states if not s.variable]
    variable_states = [s for s in states if s.variable]
    fixed_section_size = list(accumulate([s.dx for s in fixed_states]))[-1]

    accum = [f"{fixed_section_size}"] + [s.counter_var for s in variable_states]

    return f" + ".join(accum)


def boxplus_impl(cfg):
    out = []
    states = cfg['state_composition']
    fixed_states = [s for s in states if not s.variable]
    variable_states = [s for s in states if s.variable]

    def state_plus_impl(s, add_idx=False):
        if add_idx:
            idx, soft_idx = '[i]', '(i)'
        else:
            idx, soft_idx = '', ''
        if s.manifold:
            return f"xp.{s.name}{idx} = {s.name}{idx} * {s.type}::exp(dx.{s.name}{soft_idx});"
        else:
            return f"xp.{s.name}{idx} = {s.name}{idx} + dx.{s.name}{soft_idx};"

    out = []
    for s in fixed_states:
        out.append(state_plus_impl(s))
    for s in variable_states:
        out.append(f"for (size_t i = 0; i < {s.counter_var}; ++i)")
        out.append("{")
        out.append(f"    {state_plus_impl(s, True)}")
        out.append("}")
        out.append(f"xp.{s.counter_var} = {s.counter_var};")

    return "\n    ".join(out)


def self_plus_impl(cfg):
    out = []
    states = cfg['state_composition']
    fixed_states = [s for s in states if not s.variable]
    variable_states = [s for s in states if s.variable]

    def plus_impl(s, add_idx=False):
        if add_idx:
            idx, soft_idx = '[i]', '(i)'
        else:
            idx, soft_idx = '', ''
        if s.manifold:
            return f"{s.name}{idx} = {s.name}{idx} * {s.type}::exp(dx.{s.name}{soft_idx});"
        else:
            return f"{s.name}{idx} = {s.name}{idx} + dx.{s.name}{soft_idx};"

    out = []
    for s in fixed_states:
        out.append(plus_impl(s))
    for s in variable_states:
        out.append(f"for (size_t i = 0; i < {s.counter_var}; ++i)")
        out.append("{")
        out.append(f"    {plus_impl(s, True)}")
        out.append("}")
    return "\n    ".join(out)


def boxminus_impl(cfg):
    out = []
    states = cfg['state_composition']
    fixed_states = [s for s in states if not s.variable]
    variable_states = [s for s in states if s.variable]

    def minus_impl(s, add_idx=False):
        if add_idx:
            idx, soft_idx = '[i]', '(i)'
        else:
            idx, soft_idx = '', ''
        if s.manifold:
            return f"dx.{s.name}{soft_idx} = (x2.{s.name}{idx}.inverse() * {s.name}{idx}).log();"
        else:
            return f"dx.{s.name}{soft_idx} = {s.name}{idx} - x2.{s.name}{idx};"

    out = []
    for s in fixed_states:
        out.append(minus_impl(s))
    for s in variable_states:
        out.append(f"for (size_t i = 0; i < {s.counter_var}; ++i)")
        out.append("{")
        out.append(f"    {minus_impl(s, '[i]')}")
        out.append("}")
    return "\n    ".join(out)


def state_set_identity(cfg):
    states = cfg['state_composition']
    fixed_states = [s for s in states if not s.variable]
    variable_states = [s for s in states if s.variable]

    def identity_impl(s, idx=""):
        if s.manifold:
            return f"{s.name}{idx} = {s.type}::Identity();"
        else:
            return f"{s.name}{idx}.setZero();"

    out = []
    for s in fixed_states:
        out.append(identity_impl(s))
    for s in variable_states:
        out.append(f"for (size_t i = 0; i < {s.bundle_size}; ++i)")
        out.append("{")
        out.append(f"    {identity_impl(s, '[i]')}")
        out.append("}")
    return "\n    ".join(out)


def state_set_random(cfg):
    states = cfg['state_composition']
    fixed_states = [s for s in states if not s.variable]
    variable_states = [s for s in states if s.variable]

    def random_impl(s, idx=""):
        return f"{s.name}{idx}.setRandom();"

    out = []
    for s in fixed_states:
        out.append(random_impl(s))
    for s in variable_states:
        out.append(f"for (size_t i = 0; i < {s.bundle_size}; ++i)")
        out.append("{")
        out.append(f"    {random_impl(s, '[i]')}")
        out.append("}")
    return "\n    ".join(out)


def input_members(cfg):
    inputs = cfg["input"]
    out = []
    for u in inputs:
        out.append(f"{u.type} {u.name};")
    return "\n    ".join(out)


def input_dense_decl(cfg):
    states = cfg['input']
    variable_states = [s for s in states if s.variable]

    if len(variable_states) > 0:
        out_type = "Eigen::VectorXd"
        args = {f"int {s.counter_var}" for s in variable_states}
    else:
        args = {}
        out_type = f"Vec<{sum([s.dx for s in states])}>"
    return f"{out_type} dense({', '.join(args)}) const;"


def process_covariance_members(cfg):
    states = cfg['state_composition']
    fixed_states = [s for s in states if not s.variable]
    variable_states = [s for s in states if s.variable]
    out = []
    for s in fixed_states:
        out.append(f"Vec<{s.dx}> {make_camel(s.name)};")
    for s in variable_states:
        out.append(f"Vec<{s.base_dx}> {make_camel(s.name)};")
    return "\n    ".join(out)


def process_covariance_dense_decl(cfg):
    states = cfg['state_composition']
    variable_states = [s for s in states if s.variable]

    if len(variable_states) > 0:
        out_type = "Eigen::MatrixXd"
        args = {f"int {s.counter_var}" for s in variable_states}
    else:
        args = {}
        dx = sum([s.dx for s in states])
        out_type = f"Mat<{dx}, {dx}>"
    return f"{out_type} dense({', '.join(args)}) const;"


def input_covariance_members(cfg):
    states = cfg['input']
    fixed_states = [s for s in states if not s.variable]
    variable_states = [s for s in states if s.variable]
    out = []
    for s in fixed_states:
        out.append(f"Vec<{s.dx}> {make_camel(s.name)};")
    for s in variable_states:
        out.append(f"Vec<{s.base_dx}> {make_camel(s.name)};")
    return "\n    ".join(out)


def input_covariance_dense_decl(cfg):
    states = cfg['input']
    variable_states = [s for s in states if s.variable]

    if len(variable_states) > 0:
        out_type = "Eigen::MatrixXd"
        args = {f"int {s.counter_var}" for s in variable_states}
    else:
        args = {}
        dx = sum([s.dx for s in states])
        out_type = f"Mat<{dx}, {dx}>"
    return f"{out_type} dense({', '.join(args)}) const;"


def finite_state(cfg):
    states = cfg['state_composition']
    fixed_states = [s for s in states if not s.variable]
    variable_states = [s for s in states if s.variable]

    out = ["bool is_finite = true;"]
    for i, s in enumerate(fixed_states):
        out.append(f"is_finite &= mc::isFinite(x.{s.name});")
    for s in variable_states:
        out.append(f"for (const auto& sub_field : x.{s.name}){{")
        out.append("    is_finite &= mc::isFinite(sub_field);")
        out.append("}")
    out.append("return is_finite;")
    return "\n    ".join(out)


def finite_error_state(cfg):
    states = cfg['state_composition']
    fixed_states = [s for s in states if not s.variable]
    variable_states = [s for s in states if s.variable]

    out = ["bool is_finite = true;"]
    for i, s in enumerate(fixed_states):
        out.append(f"is_finite &= mc::isFinite(x.{s.name});")
    for s in variable_states:
        out.append(f"is_finite &= mc::isFinite(x.{s.name}_buf_);")
    out.append("return is_finite;")
    return "\n    ".join(out)


def finite_input(cfg):
    inputs = cfg['input']
    fixed_states = [s for s in inputs if not s.variable]
    variable_states = [s for s in inputs if s.variable]

    out = ["bool is_finite = true;"]
    for i, s in enumerate(fixed_states):
        out.append(f"is_finite &= mc::isFinite(x.{s.name});")
    for s in variable_states:
        out.append(f"for (const auto& sub_field : x.{s.name}){{")
        out.append("    is_finite &= mc::isFinite(sub_field);")
        out.append("}")
    out.append("return is_finite;")
    return "\n    ".join(out)


def finite_covariance(cfg):
    states = cfg['state_composition']

    def member_name(s1, s2):
        return f"{make_camel(s1.name)}_{make_camel(s2.name)}"

    out = ["bool is_finite = true;"]
    for r, s1 in enumerate(states):
        for c, s2 in enumerate(states):
            if r > c:
                continue
            if not s1.variable and not s2.variable:
                out.append(f"is_finite &= mc::isFinite(x.{member_name(s1, s2)});")
            else:
                out.append(f"is_finite &= mc::isFinite(x.{member_name(s1, s2)}_buf_);")
    out.append("return is_finite;")
    return "\n    ".join(out)


def finite_process_covariance(cfg):
    states = cfg['state_composition']

    out = ["bool is_finite = true;"]
    for s in states:
        out.append(f"is_finite &= mc::isFinite(x.{make_camel(s.name)});")

    out.append("return is_finite;")
    return "\n    ".join(out)


def finite_input_covariance(cfg):
    states = cfg['input']

    out = ["bool is_finite = true;"]
    for s in states:
        out.append(f"is_finite &= mc::isFinite(x.{make_camel(s.name)});")

    out.append("return is_finite;")
    return "\n    ".join(out)
    return "return true;"


def includes(cfg):
    out = []
    for inc in cfg["includes"]:
        out.append(f'#include "{inc}"')
    return "\n".join(out)


def make_dense_vector(states, name):
    fixed_states = [s for s in states if not s.variable]
    variable_states = [s for s in states if s.variable]
    out = []
    if len(variable_states) > 0:
        out_type = "Eigen::VectorXd"
        args = {f"int {s.counter_var}" for s in variable_states}
    else:
        args = {}
        dx = sum([s.dx for s in states])
        out_type = f"Vec<{dx}>"
    out.append(f"{out_type} {name}::dense({', '.join(args)}) const {{{{")
    out.append(f"    {out_type} out({{accum}});")
    accum = 0
    for s in fixed_states:
        out.append(f"    out.segment<{s.dx}>({accum}) = {s.name};")
        accum = accumulate_expr(accum, s.dx)
    for s in variable_states:
        dx = f"({s.counter_var}*{s.base_dx})"
        out.append(f"    out.segment({accum}, {dx}) = {s.name}_vec({s.counter_var});")
        accum = accumulate_expr(accum, dx)
    out.append("    return out;")
    out.append("}}\n")
    return "\n".join(out).format(accum=accum)


def from_dense(states):
    out = []
    accum = 0
    for s in states:
        getter = s.name if not s.variable else s.name + "_buf_"
        out.append(f"out.{getter} = x.segment<{s.dx}>({accum});")
        accum += s.dx
    return "\n    ".join(out)


def make_dense_matrix(row_states, col_states, name, symmetric=False, diagonal=False):
    fixed_rows_states = [s for s in row_states if not s.variable]
    variable_rows_states = [s for s in row_states if s.variable]
    fixed_col_states = [s for s in col_states if not s.variable]
    variable_col_states = [s for s in col_states if s.variable]
    if len(variable_rows_states) > 0 or len(variable_col_states) > 0:
        out_type = "Eigen::MatrixXd"
        args = {f"int {s.counter_var}" for s in variable_rows_states + variable_col_states}
    else:
        args = {}
        r_dx = sum([s.dx for s in fixed_rows_states])
        c_dx = sum([s.dx for s in fixed_col_states])
        out_type = f"Mat<{r_dx}, {c_dx}>"

    def dim(s):
        if s.variable:
            return f"({s.base_dx}*{s.counter_var})"
        else:
            return s.dx

    def impl(rs, cs, zero=False):
        if not rs.variable and not cs.variable:
            assign = f"out.block<{rs.dx}, {cs.dx}>({row_accum}, {col_accum})"
            if diagonal:
                getter = f"{make_camel(rs.name)}.asDiagonal()"
            else:
                getter = f"{make_camel(rs.name)}_{make_camel(cs.name)}"
        else:
            assign = f"out.block({row_accum}, {col_accum}, {dim(rs)}, {dim(cs)})"
            args = {s.counter_var for s in (rs, cs) if s.variable}
            if diagonal:
                getter = f"{make_camel(rs.name)}.replicate({rs.counter_var}, 1).asDiagonal()"
            else:
                getter = f"{make_camel(rs.name)}_{make_camel(cs.name)}({', '.join(args)})"
        if zero:
            return f"{assign}.setZero();"
        else:
            return f"{assign} = {getter};"

    out = [f"{out_type} {name}::dense({', '.join(args)}) const {{{{"]
    out.append(f"    {out_type} out({{row_accum}}, {{col_accum}});")
    row_accum = 0
    for r, rs in enumerate(row_states):
        col_accum = 0
        for c, cs in enumerate(col_states):
            if c < r and symmetric:
                pass
            else:
                out.append(f"    {impl(rs, cs, zero=(c != r and diagonal))}")
            col_accum = accumulate_expr(col_accum, dim(cs))
        row_accum = accumulate_expr(row_accum, dim(rs))
    if symmetric:
        out.append("    out.triangularView<Eigen::Lower>() = out.transpose();")
    out.append("    return out;")
    out.append("}}\n")
    return "\n".join(out).format(row_accum=row_accum, col_accum=col_accum)


def error_state_dense(cfg):
    return make_dense_vector(cfg["state_composition"], cfg['error_state_name'])


def input_dense(cfg):
    return make_dense_vector(cfg["input"], cfg["input_name"])


def input_from_dense(cfg):
    return make_dense_vector(cfg["input"], cfg["input_name"])


def covariance_dense(cfg):
    return make_dense_matrix(cfg["state_composition"], cfg["state_composition"], "Covariance", True)


def covariance_from_process_cov(cfg):
    states = cfg["state_composition"]
    out = []
    for s in states:
        if s.variable:
            out.append(
                f"{make_camel(s.name)}_{make_camel(s.name)}_buf_ = tile(cov.{make_camel(s.name)}.asDiagonal(), {s.bundle_size}, {s.bundle_size});"
            )
        else:
            out.append(
                f"{make_camel(s.name)}_{make_camel(s.name)} = cov.{make_camel(s.name)}.asDiagonal();"
            )
    return "\n    ".join(out)


def process_cov_dense(cfg):
    return make_dense_matrix(
        cfg["state_composition"], cfg["state_composition"], "ProcessCovariance", diagonal=True
    )


def input_cov_dense(cfg):
    return make_dense_matrix(cfg["input"], cfg["input"], "InputCovariance", diagonal=True)


def diagonal_set(states, identity=False, zero=False, random=False):
    if sum([identity, zero, random]) != 1:
        raise RuntimeError("Must set diagonal matrix to something")
    out = []
    for s in states:
        if identity:
            out.append(f"{make_camel(s.name)}.setConstant(1.0);")
        elif zero:
            out.append(f"{make_camel(s.name)}.setZero();")
        elif random:
            out.append(f"{make_camel(s.name)}.setRandom();")
    return "\n    ".join(out)


def process_cov_set_random(cfg):
    return diagonal_set(cfg["state_composition"], random=True)


def process_cov_set_zero(cfg):
    return diagonal_set(cfg["state_composition"], zero=True)


def process_cov_set_identity(cfg):
    return diagonal_set(cfg["state_composition"], identity=True)


def input_cov_set_random(cfg):
    return diagonal_set(cfg["input"], random=True)


def input_cov_set_zero(cfg):
    return diagonal_set(cfg["input"], zero=True)


def state_header(cfg):
    return STATE_HEADER_TEMPLATE.format(
        INCLUDES=includes(cfg),
        BEGIN_NAMESPACES=begin_namespaces(cfg),
        ErrorState=cfg["error_state_name"],
        State=cfg["state_name"],
        Input=cfg["input_name"],
        DOF=dof(cfg["state_composition"]),
        INPUT_DOF=dof(cfg['input']),
        USING_STATEMENTS=state_using_statements(cfg),
        ERROR_STATE_BUFFERS=error_state_buffers(cfg),
        ERROR_STATE_MEMBERS=error_state_members(cfg),
        ERROR_STATE_VECTOR_GETTERS=error_state_vector_getters(cfg),
        ERROR_STATE_DENSE_DECL=error_state_dense_decl(cfg),
        COVARIANCE_MEMBERS=covariance_members(cfg),
        COVARIANCE_BUFFERS=covariance_buffers(cfg),
        COVARIANCE_DENSE_DECL=covariance_dense_decl(cfg),
        STATE_MEMBERS=state_members(cfg),
        STATE_SIZE=state_size(cfg),
        INPUT_USING_STATEMENTS=input_using_statements(cfg),
        INPUT_MEMBERS=input_members(cfg),
        INPUT_DENSE_DECL=input_dense_decl(cfg),
        COMPUTE_CURRENT_SIZE=compute_current_size(cfg),
        PROCESS_COVARIANCE_MEMBERS=process_covariance_members(cfg),
        PROCESS_COVARIANCE_DENSE_DECL=process_covariance_dense_decl(cfg),
        INPUT_COVARIANCE_MEMBERS=input_covariance_members(cfg),
        INPUT_COVARIANCE_DENSE_DECL=input_covariance_dense_decl(cfg),
        END_NAMESPACES=end_namespaces(cfg),
        namespaces="::".join(cfg['namespaces'])
    )


def state_impl(cfg):
    return STATE_IMPL_TEMPLATE.format(
        DESTINATION_DIR=cfg["destination"],
        EKF_FILENAME=cfg["ekf_filename_base"],
        BEGIN_NAMESPACES=begin_namespaces(cfg),
        namespaces="::".join(cfg['namespaces']),
        DOF=dof(cfg['state_composition']),
        ErrorState=cfg["error_state_name"],
        Input=cfg["input_name"],
        State=cfg["state_name"],
        ERROR_STATE_SET_RANDOM=set_random(cfg),
        ERROR_STATE_SET_ZERO=set_zero(cfg),
        ERROR_STATE_CONSTANT=error_state_constant(cfg),
        ERROR_STATE_MINUS=error_state_minus(cfg),
        ERROR_STATE_PLUS=impl_plus(cfg["state_composition"]),
        INPUT_PLUS=impl_plus(cfg["input"]),
        STATE_SET_RANDOM=state_set_random(cfg),
        STATE_SET_IDENTITY=state_set_identity(cfg),
        SCALAR_MULTIPLY=scalar_multiply(cfg),
        COPY_STATE=copy_state(cfg),
        BOXPLUS_IMPL=boxplus_impl(cfg),
        SELF_PLUS_IMPL=self_plus_impl(cfg),
        BOXMINUS_IMPL=boxminus_impl(cfg),
        FINITE_STATE=finite_state(cfg),
        FINITE_INPUT=finite_input(cfg),
        FINITE_ERROR_STATE=finite_error_state(cfg),
        FINITE_COVARIANCE=finite_covariance(cfg),
        FINITE_PROCESS_COVARIANCE=finite_process_covariance(cfg),
        FINITE_INPUT_COVARIANCE=finite_input_covariance(cfg),
        ERROR_STATE_DENSE=error_state_dense(cfg),
        FROM_DENSE=from_dense(cfg['state_composition']),
        INPUT_FROM_DENSE=from_dense(cfg['input']),
        INPUT_DOF=dof(cfg['input']),
        INPUT_DENSE=input_dense(cfg),
        INPUT_RANDOM=input_random(cfg),
        INPUT_RANDOM_NORMAL=input_random_normal(cfg),
        MULTIPLY_CONSTANT=multiply_constant(cfg),
        SELF_ADD_INPUT=self_add(cfg["input"]),
        ERROR_STATE_SELF_PLUS=self_add(cfg["state_composition"]),
        INPUT_ZERO=input_zero(cfg),
        COVARIANCE_FROM_PROCESS_COV=covariance_from_process_cov(cfg),
        COVARIANCE_DENSE=covariance_dense(cfg),
        COVARIANCE_SET_RANDOM=covariance_set_random(cfg),
        COVARIANCE_IDENTITY=covariance_identity(cfg),
        PROCESS_COV_DENSE=process_cov_dense(cfg),
        PROCESS_COV_SET_RANDOM=process_cov_set_random(cfg),
        PROCESS_COV_SET_ZERO=process_cov_set_zero(cfg),
        PROCESS_COV_SET_IDENTITY=process_cov_set_identity(cfg),
        INPUT_COV_DENSE=input_cov_dense(cfg),
        INPUT_COV_SET_RANDOM=input_cov_set_random(cfg),
        INPUT_COV_SET_ZERO=input_cov_set_zero(cfg),
        END_NAMESPACES=end_namespaces(cfg)
    )
