import re

from itertools import accumulate

from core.ekf.ekf_gen.symb_algebra import reduce
from core.ekf.ekf_gen.symb_matrix import ConstBlock
from core.ekf.ekf_gen.symb_block_matrix import SymBlockMatrix
from core.ekf.ekf_gen.ekf_gen_utils import make_camel, make_snake, is_lie, get_size


def jacobian_name(row_state, col_state):
    return f"d{make_camel(row_state).capitalize()}_d{make_camel(col_state).capitalize()}"


symbol_varname = {"transpose": "transpose"}


def create_symbolic_dynamics_jacobian(cfg):
    keys = [k for k, v in cfg["state_composition"].items()]
    input_keys = [k for k, v in cfg["input"].items()]
    sizes = [get_size(v)[1] for k, v in cfg["state_composition"].items()]
    input_sizes = [get_size(v)[1] for _, v in cfg["input"].items()]

    mat = []
    for r, rstate in enumerate(keys):
        row = []
        for c, cstate in enumerate(keys):
            if cstate in cfg["dynamics"][rstate][0]:
                name = jacobian_name(rstate, cstate)
                row.append(ConstBlock(name, sizes[r], sizes[c]))
                symbol_varname[name] = f"dxdx.{name}"
            else:
                row.append(ConstBlock("0", sizes[r], sizes[c]))
        mat.append(row)
    A = SymBlockMatrix(mat)

    mat = []
    for r, rstate in enumerate(keys):
        row = []
        for c, cstate in enumerate(input_keys):
            if cstate in cfg["dynamics"][rstate][1]:
                name = jacobian_name(rstate, cstate)
                row.append(ConstBlock(name, sizes[r], input_sizes[c]))
                symbol_varname[name] = f"dxdu.{name}"
            else:
                row.append(ConstBlock('0', sizes[r], input_sizes[c]))
        mat.append(row)
    B = SymBlockMatrix(mat)
    return A, B


def create_symbolic_covariance(cfg):
    keys = [k for k, v in cfg["state_composition"].items()]
    sizes = [get_size(v)[1] for k, v in cfg["state_composition"].items()]
    index = list(accumulate(sizes))
    index.insert(0, 0)

    mat = []
    for r, rstate in enumerate(keys):
        row = []
        for c, cstate in enumerate(keys):
            name = f"cov_{r}_{c}"
            row.append(ConstBlock(name, sizes[r], sizes[c]))
            symbol_varname[name] = f"cov_.block<{sizes[r]}, {sizes[c]}>({index[r]}, {index[c]})"
        mat.append(row)
    P = SymBlockMatrix(mat)
    return P


def create_symbolic_input_cov(cfg):
    input_keys = [k for k, v in cfg["input"].items()]
    input_sizes = [get_size(v)[1] for _, v in cfg["input"].items()]
    index = list(accumulate(input_sizes))
    index.insert(0, 0)

    mat = []
    for r, rstate in enumerate(input_keys):
        row = []
        for c, cstate in enumerate(input_keys):
            if c == r:
                name = f"Qu_{r}_{c}"
                row.append(ConstBlock(name, input_sizes[r], input_sizes[c]))
                symbol_varname[
                    name] = f"Qu.block<{input_sizes[r]}, {input_sizes[c]}>({index[r]}, {index[c]})"
            else:
                row.append(ConstBlock('0', input_sizes[r], input_sizes[c]))
        mat.append(row)
    Qu = SymBlockMatrix(mat)
    return Qu


def create_symbolic_process_cov(cfg):
    keys = [k for k, v in cfg["state_composition"].items()]
    sizes = [get_size(v)[1] for k, v in cfg["state_composition"].items()]
    index = list(accumulate(sizes))
    index.insert(0, 0)

    mat = []
    for r, rstate in enumerate(keys):
        row = []
        for c, cstate in enumerate(keys):
            if c == r:
                name = f"Qx_{r}_{c}"
                row.append(ConstBlock(name, sizes[r], sizes[c]))
                symbol_varname[name] = f"Qx.block<{sizes[r]}, {sizes[c]}>({index[r]}, {index[c]})"
            else:
                row.append(ConstBlock('0', sizes[r], sizes[c]))
        mat.append(row)
    Qx = SymBlockMatrix(mat)
    return Qx


def create_symbolic_identity(cfg):
    keys = [k for k, v in cfg["state_composition"].items()]
    sizes = [get_size(v)[1] for k, v in cfg["state_composition"].items()]
    index = list(accumulate(sizes))
    index.insert(0, 0)

    mat = []
    for r, rstate in enumerate(keys):
        row = []
        for c, cstate in enumerate(keys):
            if c == r:
                row.append(ConstBlock("I", sizes[r], sizes[c]))
                key = f"I_{sizes[r]}x{sizes[c]}"
                symbol_varname[key] = f"Eigen::Matrix<double, {sizes[r]}, {sizes[c]}::Identity()"
            else:
                row.append(ConstBlock("0", sizes[r], sizes[c]))
        mat.append(row)
    I = SymBlockMatrix(mat)
    return I


def commit_mat(matrix, var):
    def make_type(expr):
        return f"const Eigen::Matrix<double, {expr.rows}, {expr.cols}>"

    def impl_expr(expr):
        return re.sub(r"([A-Za-z][a-zA-Z_0-9]+)", r"{\1}", reduce(expr).render())

    new_mat = []
    out = []
    for r in range(matrix.rows):
        new_row = []
        for c in range(matrix.cols):
            expr = matrix[r, c]
            if expr.is_zero:
                new_row.append(ConstBlock("0", expr.rows, expr.cols))
                continue
            if expr.is_identity:
                new_row.append(ConstBlock("I", expr.rows, expr.cols))
                continue

            new_row.append(ConstBlock(f"{var}_{r}_{c}", expr.rows, expr.cols))
            name = f"{var}_{r}_{c}"
            symbol_varname[name] = name
            out.append(f"{make_type(expr)} {{{name}}} = {impl_expr(expr)};")
        new_mat.append(new_row)
    out[-1] += "\n"
    return out, SymBlockMatrix(new_mat)


def commit_matrix_change(matrix, var, compare, symmetric=False):
    def impl_expr(expr):
        return re.sub(r"([A-Za-z][a-zA-Z_0-9]+)", r"{\1}", reduce(expr).render())

    out = []
    for r in range(matrix.rows):
        for c in range(matrix.cols):
            if symmetric and c < r:
                continue
            expr = matrix[r, c]
            compare_expr = compare[r, c]
            if expr == compare_expr:
                continue
            out.append(f"{{{var}_{r}_{c}}} = {impl_expr(expr)};")
    out[-1] += "\n"
    return out


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
    P = create_symbolic_covariance(cfg)
    Qu = create_symbolic_input_cov(cfg)
    Qx = create_symbolic_process_cov(cfg)
    I = create_symbolic_identity(cfg)

    dt = ConstBlock("dt", 1, 1)
    symbol_varname["dt"] = "dt"
    half = ConstBlock("0.5", 1, 1)
    Abar = I + A * dt
    Bbar = (I + A * dt * half) * B * dt

    abar_impl, Abar = commit_mat(Abar, "Abar")
    bbar_impl, Bbar = commit_mat(Bbar, "Bbar")
    bqbt_impl, BqBT = commit_mat(Bbar * Qu * Bbar.T, "bqbt")

    P_up = Abar * P * Abar.T + BqBT + Qx
    P_up_impl = commit_matrix_change(P_up, "cov", P, symmetric=True)

    out = abar_impl + bbar_impl + bqbt_impl + P_up_impl

    template = "\n    ".join(out)

    # Substitute the actual variables
    impl = template.format(**symbol_varname)

    return impl


def create_symbolic_measurement_jacobian(cfg, key):
    state_keys = [k for k, v in cfg["state_composition"].items()]
    state_sizes = [get_size(v)[1] for k, v in cfg["state_composition"].items()]

    meas_cfg = cfg["measurements"][key]
    meas_keys = meas_cfg["states"]
    meas_size = meas_cfg["size"]

    dzdx = []
    for c, cstate in enumerate(state_keys):
        if cstate in meas_keys:
            name = jacobian_name(key, cstate)
            dzdx.append(ConstBlock(name, meas_size, state_sizes[c]))
            symbol_varname[name] = f"dzdx.{name}"
        else:
            dzdx.append(ConstBlock("0", meas_size, state_sizes[c]))
    return SymBlockMatrix([dzdx])


def compute_innovation(cfg, key):
    meas_cfg = cfg["measurements"][key]
    meas_size = meas_cfg["size"]
    H = create_symbolic_measurement_jacobian(cfg, key)
    P = create_symbolic_covariance(cfg)
    R = SymBlockMatrix([[ConstBlock("R", meas_size, meas_size)]])

    innov = H * P * H.T + R

    symbol_varname["innov_inv"] = "innov_inv"

    innov_impl, innov = commit_mat(innov, "innov_inv")
    innov_impl.append(
        f"const Eigen::Matrix<double, {meas_size}, {meas_size}> innov = {{innov_inv}}.inverse();"
    )
    innov_impl = "\n    ".join(innov_impl)
    innov_impl = innov_impl.format(**symbol_varname)

    # Compute the Kalman Gain
    K = P * H.T * innov
    kalman_impl, K = commit_mat(K, "K")
    kalman_impl = "\n    ".join(kalman_impl)
    kalman_impl = kalman_impl.format(**symbol_varname)

    # Compute Covariance Update
    I = create_symbolic_identity(cfg)
    A = (I - K * H).reduce()

    A_impl, A = commit_mat(A, "A")
    P_up = A * P * A.T + K * R * K.T
    P_up_impl = commit_matrix_change(P_up, "cov", P, symmetric=True)
    update_impl = A_impl + P_up_impl
    update_impl = "\n    ".join(update_impl)
    update_impl = update_impl.format(**symbol_varname)

    return innov_impl, kalman_impl, update_impl
