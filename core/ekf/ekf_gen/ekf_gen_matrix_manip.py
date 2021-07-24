import re

from itertools import accumulate

from core.ekf.ekf_gen.symb_algebra import reduce
from core.ekf.ekf_gen.symb_matrix import ConstBlock
from core.ekf.ekf_gen.symb_block_matrix import SymBlockMatrix
from core.ekf.ekf_gen.ekf_gen_utils import State, make_camel, make_snake, jac_member_name, accumulate_expr

symbol_varname = {"transpose": "transpose"}


def dim(state):
    if state.variable:
        # return f"{state.counter_var} * {state.base_dx}"
        # return state.counter_var
        return re.sub("num_", "size_", state.counter_var)
    else:
        return state.dx


def block_expr(r_expr, c_expr, rstate, cstate):
    if any((isinstance(s, str) for s in [dim(rstate), dim(cstate)])):
        return f".block({r_expr}, {c_expr}, {dim(rstate)}, {dim(cstate)})"
    else:
        return f".block<{dim(rstate)}, {dim(cstate)}>({r_expr}, {c_expr})"


def create_symbolic_error_state(cfg):
    states = cfg["state_composition"]

    mat = []
    for r, rstate in enumerate(states):
        if rstate.variable:
            name = f"dx_{rstate.name}"
            symbol_varname[name] = f"dx.{rstate.name}_vec({rstate.counter_var})"
        else:
            name = f"dx_{rstate.name}"
            symbol_varname[name] = f"dx.{rstate.name}"
        mat.append([ConstBlock(name, dim(rstate), 1, rstate.dx, 1)])
    return SymBlockMatrix(mat)


def create_symbolic_dynamics_jacobian(cfg):
    states = cfg["state_composition"]
    inputs = cfg["input"]

    mat = []
    for r, rstate in enumerate(states):
        row = []
        for c, cstate in enumerate(states):
            if cstate.name in cfg["dynamics"][rstate.name][0]:
                name = jac_member_name(rstate, cstate)
                row.append(ConstBlock(name, dim(rstate), dim(cstate), rstate.dx, cstate.dx))
                if rstate.variable or cstate.variable:
                    args = {s.counter_var for s in [rstate, cstate] if s.variable}
                    symbol_varname[name] = f"dxdx.{name}({', '.join(args)})"
                else:
                    symbol_varname[name] = f"dxdx.{name}"
            else:
                row.append(ConstBlock("0", dim(rstate), dim(cstate), rstate.dx, cstate.dx))
        mat.append(row)
    A = SymBlockMatrix(mat)

    mat = []
    for r, rstate in enumerate(states):
        row = []
        for c, cstate in enumerate(inputs):
            if cstate.name in cfg["dynamics"][rstate.name][1]:
                name = jac_member_name(rstate, cstate)
                row.append(ConstBlock(name, dim(rstate), dim(cstate), rstate.dx, cstate.dx))
                if rstate.variable or cstate.variable:
                    args = {s.counter_var for s in [rstate, cstate] if s.variable}
                    symbol_varname[name] = f"dxdu.{name}({', '.join(args)})"
                else:
                    symbol_varname[name] = f"dxdu.{name}"
            else:
                row.append(ConstBlock("0", dim(rstate), dim(cstate), rstate.dx, cstate.dx))
        mat.append(row)
    B = SymBlockMatrix(mat)
    return A, B


def create_symbolic_covariance(cfg, var_name):
    states = cfg["state_composition"]
    mat = []

    def block_name(rs, cs):
        if not rs.variable and not cs.variable:
            return f"{make_camel(rs.name)}_{make_camel(cs.name)}"
        else:
            args = ', '.join({f"{s.counter_var}" for s in [rs, cs] if s.variable})
            return f"{make_camel(rs.name)}_{make_camel(cs.name)}({args})"

    for r, rstate in enumerate(states):
        row = []
        for c, cstate in enumerate(states):
            if c < r:
                expr = f"{var_name}.{block_name(cstate, rstate)}.transpose()"
            else:
                expr = f"{var_name}.{block_name(rstate, cstate)}"
            name = f"{var_name}_{r}_{c}"
            row.append(ConstBlock(name, dim(rstate), dim(cstate), rstate.dx, cstate.dx))
            symbol_varname[name] = expr
        mat.append(row)
    P = SymBlockMatrix(mat)
    return P


def create_symbolic_input_cov(cfg):
    inputs = cfg["input"]
    r_expr = 0
    c_expr = 0
    mat = []
    for r, rstate in enumerate(inputs):
        row = []
        for c, cstate in enumerate(inputs):
            if c == r:
                name = f"Qu_{r}_{c}"
                row.append(ConstBlock(name, dim(rstate), dim(cstate), rstate.dx, cstate.dx))
                symbol_varname[name] = f"Qu.{make_camel(rstate.name)}.asDiagonal()"
            else:
                row.append(ConstBlock('0', dim(rstate), dim(cstate), rstate.dx, cstate.dx))
            accumulate_expr(c_expr, dim(cstate))
        accumulate_expr(r_expr, dim(rstate))
        mat.append(row)
    Qu = SymBlockMatrix(mat)
    return Qu


def create_symbolic_process_cov(cfg):
    states = cfg["state_composition"]
    r_expr = 0
    c_expr = 0
    mat = []

    for r, rstate in enumerate(states):
        row = []
        for c, cstate in enumerate(states):
            if c == r:
                name = f"Qx_{r}_{c}"
                row.append(ConstBlock(name, dim(rstate), dim(cstate), rstate.dx, cstate.dx))
                if rstate.variable:
                    symbol_varname[
                        name
                    ] = f"Qx.{make_camel(rstate.name)}.replicate({rstate.counter_var}, 1).asDiagonal()"
                else:
                    symbol_varname[name] = f"Qx.{make_camel(rstate.name)}.asDiagonal()"
            else:
                row.append(ConstBlock('0', dim(rstate), dim(cstate), rstate.dx, cstate.dx))
            accumulate_expr(c_expr, dim(cstate))
        accumulate_expr(r_expr, dim(rstate))
        mat.append(row)
    Qx = SymBlockMatrix(mat)
    return Qx


def create_symbolic_identity(cfg):
    states = cfg["state_composition"]
    mat = []

    def identity_block(state):
        if state.variable:
            return f"Eigen::MatrixXd::Identity({dim(state)}, {dim(state)})"
        else:
            return f"Mat<{dim(state)},{dim(state)}>::Identity()"

    for r, rstate in enumerate(states):
        row = []
        for c, cstate in enumerate(states):
            if c == r:
                row.append(ConstBlock("I", dim(rstate), dim(cstate), rstate.dx, cstate.dx))
                key = f"I_{dim(rstate)}x{dim(cstate)}"

                symbol_varname[key] = identity_block(rstate)
            else:
                row.append(ConstBlock('0', dim(rstate), dim(cstate), rstate.dx, cstate.dx))
        mat.append(row)
    Qx = SymBlockMatrix(mat)
    return Qx


def make_type(expr):
    return f"Mat<{expr.max_rows},{expr.max_cols}>"


def commit_mat_inplace_modify(matrix, var, modify_type='-', symmetric=False):
    def impl_expr(expr):
        return re.sub(r"([A-Za-z][a-zA-Z_0-9]+)", r"{\1}", reduce(expr).render())

    out = []
    for r in range(matrix.rows):
        for c in range(matrix.cols):
            if symmetric and c < r:
                continue
            expr = matrix[r, c]
            if not expr.is_zero:
                if expr.is_variable():
                    sizes = {f"({s} > 0)" for s in (expr.rows, expr.cols) if isinstance(s, str)}
                    out.append(f"if ({' && '.join(sizes)}) {{{{")
                    out.append(f"    {impl_expr(var[r,c])} {modify_type}= {impl_expr(expr)};")
                    out.append("}}")
                else:
                    out.append(f"{impl_expr(var[r,c])} {modify_type}= {impl_expr(expr)};")
    out[-1] += "\n"
    return out, var


def commit_mat(matrix, var, symmetric=False, dense=False, const=True, already_checked_size=[]):
    def impl_expr(expr):
        return re.sub(r"([A-Za-z][a-zA-Z_0-9]+)", r"{\1}", reduce(expr).render())

    out = []
    if isinstance(var, SymBlockMatrix):
        for r in range(matrix.rows):
            for c in range(matrix.cols):
                if symmetric and c < r:
                    continue
                expr = matrix[r, c]
                if expr.is_variable():
                    sizes = {f"({s} > 0)" for s in (expr.rows, expr.cols) if isinstance(s, str)}
                    out.append(f"if ({' && '.join(sizes)}) {{{{")
                    out.append(f"    {impl_expr(var[r,c])} = {impl_expr(expr)};")
                    out.append("}}")
                else:
                    out.append(f"{impl_expr(var[r,c])} = {impl_expr(expr)};")
        out[-1] += "\n"
        return out, var
    else:
        new_mat = []
        for r in range(matrix.rows):
            new_row = []
            for c in range(matrix.cols):
                if symmetric and r < c:
                    continue
                expr = matrix[r, c]
                key = f"{var}_{r}_{c}"
                if expr.is_zero:
                    new_row.append(
                        ConstBlock("0", expr.rows, expr.cols, expr.max_rows, expr.max_cols)
                    )
                    continue
                if expr.is_identity:
                    new_row.append(
                        ConstBlock("I", expr.rows, expr.cols, expr.max_rows, expr.max_cols)
                    )
                    continue

                new_row.append(ConstBlock(key, expr.rows, expr.cols, expr.max_rows, expr.max_cols))

                if not dense:
                    name = key
                else:
                    name = f"{var}"
                if expr.is_variable():
                    # Need to allocate and then only use part
                    expr_text = make_type(expr)
                    symbol_varname[f"{name}_init"] = name
                    out.append(f"{expr_text} {{{name}_init}};")
                    sizes = {
                        f"({s} > 0)"
                        for s in (expr.rows, expr.cols)
                        if isinstance(s, str) and s not in already_checked_size
                    }
                    if len(sizes) > 0:
                        out.append(f"if ({' && '.join(sizes)}) {{{{")
                    sub_name = f"{name}.block(0, 0, {expr.rows}, {expr.cols})"
                    out.append(f"{{{key}}} = {impl_expr(expr)};")
                    if len(sizes) > 0:
                        out[-1] = "    " + out[-1]
                        out.append("}}")
                else:
                    const = "const " if const else ""
                    out.append(f"{const}{make_type(expr)} {{{key}}} = {impl_expr(expr)};")
                    sub_name = name
                symbol_varname[key] = sub_name
            new_mat.append(new_row)
        return out, SymBlockMatrix(new_mat)


def to_dense_matrix(matrix, var):
    def impl_expr(expr):
        return re.sub(r"([A-Za-z][a-zA-Z_0-9]+)", r"{\1}", reduce(expr).render())

    out = [f"const Eigen::Matrix<double, {matrix.total_rows}, {matrix.total_cols}> {var} = ("]
    for r in range(matrix.rows):
        row = []
        for c in range(matrix.cols):
            expr = matrix[r, c]
            if expr.is_zero:
                row.append(f"Eigen::Matrix<double, {expr.rows}, {expr.cols}>::Zero()")
                continue
            if expr.is_identity:
                row.append(f"Eigen::Matrix<double, {expr.rows}, {expr.cols}>::Identity()")
                continue
            row.append(f"{impl_expr(expr)}")
        out.append(" << ".join(row))
    out[-1] += "\n"
    return out


def block_sparse_propagate_covariance(cfg):
    A, B = create_symbolic_dynamics_jacobian(cfg)
    Pin = create_symbolic_covariance(cfg, "cov")
    Pout = create_symbolic_covariance(cfg, "cov_out")
    Qu = create_symbolic_input_cov(cfg)
    Qx = create_symbolic_process_cov(cfg)
    I = create_symbolic_identity(cfg)    # noqa

    dt = ConstBlock("dt", 1, 1)
    symbol_varname["dt"] = "dt"
    half = ConstBlock("0.5", 1, 1)
    Abar = I + A * dt + A * A * dt * dt * half
    Bbar = (I + A * dt * half) * B * dt

    abar_impl, Abar = commit_mat(Abar, "Abar")
    tmp = "\n    ".join(abar_impl)
    ABAR = tmp.format(**symbol_varname)

    bbar_impl, Bbar = commit_mat(Bbar, "Bbar")
    tmp = "\n    ".join(bbar_impl)
    BBAR = tmp.format(**symbol_varname)

    Pout_impl, _ = commit_mat(Abar * Pin * Abar.T + Bbar * Qu * Bbar.T, Pout, symmetric=True)
    ProcNoise_impl, _ = commit_mat_inplace_modify(Qx, Pout, "+", True)

    prop_cov_impl = Pout_impl + ProcNoise_impl
    tmp = "\n    ".join(prop_cov_impl)
    PROPAGATE_COVARIANCE = tmp.format(**symbol_varname)

    return {
        "ABAR": ABAR,
        "BBAR": BBAR,
        "PROPAGATE_COVARIANCE": PROPAGATE_COVARIANCE,
    }


def create_symbolic_measurement_jacobian(cfg, key):
    states = cfg["state_composition"]
    meas_cfg = cfg["measurements"][key]

    dzdx = []
    sr = State(None, meas_cfg["type"])
    meas_state = State(key, meas_cfg["type"])
    for sc in states:
        if sc.name in meas_cfg["states"]:
            name = f"{jac_member_name(sr, sc)}"
            dzdx.append(ConstBlock(name, dim(meas_state), dim(sc), sr.dx, sc.dx))
            if meas_state.variable or sc.variable:
                args = {s.counter_var for s in (meas_state, sc) if s.variable}
                symbol_varname[name] = f"dzdx.{name}({','.join(args)})"
            else:
                symbol_varname[name] = f"dzdx.{name}"
        else:
            dzdx.append(ConstBlock("0", dim(meas_state), dim(sc), sr.dx, sc.dx))
    return SymBlockMatrix([dzdx])


def create_symbolic_meas_covariance(cfg, key, name):
    symbol_varname[name] = f"{name}.asDiagonal()"

    meas_cfg = cfg["measurements"][key]
    meas_state = State(None, meas_cfg["type"])
    return SymBlockMatrix([[
        ConstBlock(name, dim(meas_state), dim(meas_state), meas_state.dx, meas_state.dx)
    ]])


def project_cov(cfg):
    args = ''.join(f", snap->x.{s.counter_var}" for s in cfg['state_composition'] if s.variable)
    return f"Meas::Pht::projectCovariance(snap->cov, dzdx{args})"


def sparse_meas_update(cfg, key):
    meas_cfg = cfg["measurements"][key]
    meas_state = State(key, meas_cfg["type"])
    dim_meas = dim(meas_state)
    P = create_symbolic_covariance(cfg, "cov")
    H = create_symbolic_measurement_jacobian(cfg, key)
    dx = create_symbolic_error_state(cfg)
    res = SymBlockMatrix([[ConstBlock("res", dim_meas, 1, meas_state.dx, 1)]])
    R = SymBlockMatrix([[ConstBlock("{R}", dim_meas, dim_meas, meas_state.dx, meas_state.dx)]])
    Sinv = SymBlockMatrix([[ConstBlock("Sinv", dim_meas, dim_meas, meas_state.dx, meas_state.dx)]])
    symbol_varname[
        "R"
    ] = "R.asDiagonal()" if not meas_state.variable else f"R.replicate({meas_state.counter_var}, 1).asDiagonal()"
    symbol_varname[
        "Sinv"] = "Sinv" if not meas_state.variable else f"Sinv.block(0, 0, {dim_meas}, {dim_meas})"
    symbol_varname["res"] = "res" if not meas_state.variable else f"res.head({dim_meas})"

    # Compute the covariance projection
    PHT_impl, PHT = commit_mat(P * H.T, "PHT", already_checked_size=[H[0, 0].rows])
    tmp = "\n    ".join(PHT_impl)
    SPARSE_PHT = tmp.format(**symbol_varname)

    HPHT_impl, HPHT = commit_mat(H * PHT, "HPHT", dense=True, const=False)
    S_impl, _ = commit_mat_inplace_modify(R, HPHT, '+', symmetric=True)
    HPHT_impl += S_impl
    if not Sinv[0, 0].is_variable():
        HPHT_impl.append(f"const {make_type(Sinv[0,0])} Sinv = HPHT.inverse();")
    else:
        HPHT_impl.append(f"{make_type(Sinv[0,0])} Sinv;")
        block_args = f"0,0,{meas_state.counter_var}*{meas_state.base_dx},{meas_state.counter_var}*{meas_state.base_dx}"
        HPHT_impl.append(f"Sinv.block({block_args}) = HPHT.block({block_args}).inverse();")
    tmp = "\n    ".join(HPHT_impl)
    SPARSE_INNOVATION = tmp.format(**symbol_varname)

    # Compute Kalman Gain
    K_impl, K = commit_mat(PHT * Sinv, "K", already_checked_size=[H[0, 0].rows])
    tmp = "\n        ".join(K_impl)
    SPARSE_KALMAN_GAIN = tmp.format(**symbol_varname)

    # State Update
    dx_impl, _ = commit_mat(K * res, dx)
    tmp = "\n        ".join(dx_impl)
    SPARSE_COMPUTE_ERROR_STATE = tmp.format(**symbol_varname)

    # Covariance update
    Pout_impl, _ = commit_mat_inplace_modify(K * PHT.T, P, '-', symmetric=True)
    tmp = "\n        ".join(Pout_impl)
    SPARSE_COVARIANCE_UPDATE = tmp.format(**symbol_varname)

    return {
        "SPARSE_PHT": SPARSE_PHT,
        "SPARSE_INNOVATION": SPARSE_INNOVATION,
        "SPARSE_KALMAN_GAIN": SPARSE_KALMAN_GAIN,
        "SPARSE_COMPUTE_ERROR_STATE": SPARSE_COMPUTE_ERROR_STATE,
        "SPARSE_COVARIANCE_UPDATE": SPARSE_COVARIANCE_UPDATE,
    }
