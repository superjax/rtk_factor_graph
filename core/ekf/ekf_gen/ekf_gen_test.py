import re

from core.ekf.ekf_gen.ekf_gen_utils import accumulate_expr, begin_namespaces, end_namespaces, make_snake, make_camel, State, test_eq_compare
from core.ekf.ekf_gen.ekf_gen_templates import TEST_TEMPLATE
from core.ekf.ekf_gen.ekf_gen_matrix_manip import block_sparse_propagate_covariance, create_symbolic_dynamics_jacobian, create_symbolic_identity, sparse_meas_update, create_symbolic_covariance, create_symbolic_measurement_jacobian, dim
from core.ekf.ekf_gen.symb_matrix import ConstBlock
from core.ekf.ekf_gen.symb_block_matrix import SymBlockMatrix


def size_var(s):
    return re.sub("num_", "size_", s.counter_var)


def size_variables(states):
    out = []
    for s in states:
        if s.variable:
            out.append(f"const int {size_var(s)} = {s.counter_var} * {s.base_dx};")
    return "\n    ".join(out)


def dense_args(states):
    return ', '.join(s.counter_var for s in states if s.variable)


def abar_test(cfg):
    A, _ = create_symbolic_dynamics_jacobian(cfg)
    dt = ConstBlock("dt", 1, 1)
    half = ConstBlock("0.5", 1, 1)
    I = create_symbolic_identity(cfg)    # noqa
    Abar = (I + A * dt + A * A * dt * dt * half).reduce()
    return sparse_mat_eq(Abar, "Abar_d", "Abar")


def sparse_mat_eq(mat, dense, var, already_checked_sizes=[]):
    out = []
    row_accum = 0
    for r in range(mat.rows):
        col_accum = 0
        for c in range(mat.cols):
            block = mat[r, c]
            if not block.is_zero and not block.is_identity:
                sparse_var = f"{var}_{r}_{c}.block(0, 0, {block.rows}, {block.cols})"
                dense_getter = f"{dense}.block({row_accum}, {col_accum}, {block.rows}, {block.cols})"
                if block.is_variable():
                    sizes = {
                        f"({s} > 0)"
                        for s in (block.rows, block.cols)
                        if isinstance(s, str) and s not in already_checked_sizes
                    }
                    if len(sizes) > 0:
                        out.append(f"if ({' && '.join(sizes)}) {{")
                    out.append(f"EXPECT_FALSE({dense_getter}.isZero());")
                    out.append(f"MAT_EQ({sparse_var}, {dense_getter});")
                    if len(sizes) > 0:
                        out[-1] = "    " + out[-1]
                        out[-2] = "    " + out[-2]
                        out.append("}")
                else:
                    out.append(f"EXPECT_FALSE({dense_getter}.isZero());")
                    out.append(f"MAT_EQ({sparse_var}, {dense_getter});")

            col_accum = accumulate_expr(col_accum, block.cols)
        row_accum = accumulate_expr(row_accum, block.rows)
    return "\n    ".join(out)


def bbar_test(cfg):
    A, B = create_symbolic_dynamics_jacobian(cfg)
    dt = ConstBlock("dt", 1, 1)
    half = ConstBlock("0.5", 1, 1)
    I = create_symbolic_identity(cfg)    # noqa
    Bbar = ((I + A * dt * half) * B * dt).reduce()
    return sparse_mat_eq(Bbar, "Bbar_d", "Bbar")


def dedent(text):
    return "\n".join((line[4:] if i > 0 else line for i, line in enumerate(text.split("\n"))))


SPARSE_PROPAGATE_TEST_TEMPLATE = """    {SIZE_VARIABLES}
    const auto dxdx = StateJac::Random();
    const auto Ad = dxdx.dense({STATE_JAC_ARGS});
    const auto dxdu = InputJac::Random();
    const auto Bd = dxdu.dense({INPUT_JAC_ARGS});
    const auto cov = Covariance::Random();
    const auto Pd = cov.dense({COVARIANCE_ARGS});
    const auto Qx = ProcessCovariance::Random();
    const auto Qx_d = Qx.dense({PROCESS_COV_ARGS});
    const auto Qu = InputCovariance::Random();
    const auto Qu_d = Qu.dense({INPUT_COV_ARGS});
    Covariance cov_out;

    double dt = 0.2;

    // Abar = I + A*dt + 0.5*A^2*dt^2 ≈ exp(A*dt)
    {ABAR}
    const decltype(Ad) Abar_d = decltype(Ad)::Identity(Ad.rows(), Ad.cols()) + Ad * dt + 0.5*Ad*Ad*dt*dt;
    {ABAR_TEST}

    // Bbar = B*dt + 0.5*AB*dt^2
    {BBAR}
    const decltype(Bd) Bbar_d = Bd*dt + 0.5 * Ad *Bd * dt*dt;
    {BBAR_TEST}

    // P = Abar ⋅ P ⋅ Abarᵀ + Bbar ⋅ Qu ⋅ Bbarᵀ + Qx
    {PROPAGATE_COVARIANCE}
    decltype(Pd) Pout_d = Abar_d * Pd * Abar_d.transpose() + Bbar_d * Qu_d * Bbar_d.transpose() + Qx_d;
    EXPECT_FALSE(Pout_d.isZero());
    MAT_EQ(Pout_d, cov_out.dense({COVARIANCE_ARGS}));
}}
"""


def num_var(type, states):
    out = []
    for s in states:
        if s.variable:
            if type == "max":
                out.append(f"int {s.counter_var} = {s.bundle_size};")
            elif type == "min":
                out.append(f"int {s.counter_var} = 0;")
            elif type == "mid":
                out.append(f"int {s.counter_var} = {round(s.bundle_size/2.0)};")
            else:
                raise RuntimeError("must specify min, max, or mid")
    return "    " + "\n    ".join(out)


def sparse_propagate_test(cfg):
    states = cfg["state_composition"]
    inputs = cfg["input"]

    def impl_test():
        return SPARSE_PROPAGATE_TEST_TEMPLATE.format(
            **block_sparse_propagate_covariance(cfg),
            EkfName=cfg["kalman_filter_class"],
            SIZE_VARIABLES=size_variables(states + inputs),
            STATE_JAC_ARGS=dense_args(states),
            INPUT_JAC_ARGS=dense_args(states + inputs),
            COVARIANCE_ARGS=dense_args(states),
            PROCESS_COV_ARGS=dense_args(states),
            INPUT_COV_ARGS=dense_args(inputs),
            ABAR_TEST=abar_test(cfg),
            BBAR_TEST=bbar_test(cfg),
        )

    args = {f'int {s.counter_var}' for s in states + inputs if s.variable}

    out = []
    if len(args) > 0:
        calling_args = {f'{s.counter_var}' for s in states + inputs if s.variable}
        out.append(f"void do_sparse_cov_test({', '.join(args)}) {{ ")
        out.append(impl_test())

        for n in ["max", "min", 'mid']:
            out.append(
                f"TEST({cfg['kalman_filter_class']}, SparseCovarianceProp{n.capitalize()}) {{"
            )
            out.append(num_var(n, states + inputs))
            out.append(f"    do_sparse_cov_test({', '.join(calling_args)});")
            out.append("}\n")
    else:
        out.append(f"TEST({cfg['kalman_filter_class']}, SparseCovarianceProp) {{")
        out.append(impl_test())
    return '\n'.join(out)


def gen_test(cfg):
    return TEST_TEMPLATE.format(
        DESTINATION_DIR=cfg["destination"],
        EKF_FILENAME=make_snake(cfg["kalman_filter_class"]),
        BEGIN_NAMESPACES=begin_namespaces(cfg),
        SPARSE_PROPAGATE_TEST=sparse_propagate_test(cfg),
        END_NAMESPACES=end_namespaces(cfg),
        SPARSE_UPDATE_TEST=sparse_update_test(cfg),
        **boxplus_rules_test(cfg)
    )


def sinv_test(meas_state):
    if meas_state.variable:
        out = []
        out.append(f"if ({meas_state.counter_var} > 0 ){{")
        block_args = f"0,0,{meas_state.counter_var}*{meas_state.base_dx},{meas_state.counter_var}*{meas_state.base_dx}"
        out.append(f"        MAT_EQ(Sinv.block({block_args}), Sinv_d);")
        out.append("    }")
        return "\n".join(out)

    else:
        return "MAT_EQ(Sinv, Sinv_d);"


def bail_if_variable_meas_empty(cfg, key):
    meas_cfg = cfg["measurements"][key]
    meas_state = State(key, meas_cfg['type'])

    if meas_state.variable:
        out = []
        out.append(f"if ({meas_state.counter_var} == 0) {{")
        out.append("    return;")
        out.append("}")
        return "\n    ".join(out)
    else:
        return ""


SPARSE_UPDATE_TEST = """    {SIZE_VARIABLES}
    Covariance cov = Covariance::Random();
    const auto P_d = cov.dense({COVARIANCE_ARGS});
    const {Meas}::Jac dzdx = {Meas}::Jac::Random();
    const auto H_d = dzdx.dense({MEAS_JAC_ARGS});
    const {Meas}::Covariance R = {Meas}::Covariance::Random();
    const Eigen::MatrixXd R_d = R{REPLICATE_R}.asDiagonal();
    const {Meas}::Residual res = {Meas}::Residual::Random();
    const Eigen::VectorXd res_d = res{RESIDUAL_HEAD};

    {BAIL_IF_VARIABLE_MEAS_EMPTY}

    //---------------------------------//
    //           PHT  CHECK            //
    //---------------------------------//
    // Compute a block-sparse P⋅Hᵀ
    {SPARSE_PHT}

    // Check the Computation of PHT versus the fully-dense computation
    const Eigen::MatrixXd PHT_d = P_d * H_d.transpose();
    {PHT_TEST}

    //---------------------------------//
    //       INNOVATION CHECK          //
    //---------------------------------//
    // Compute the Innovation S⁻¹ = (H⋅P⋅Hᵀ+R)⁻¹
    {SPARSE_INNOVATION}

    // Compare with the dense computation
    const Eigen::MatrixXd Sinv_d = (H_d * PHT_d + R_d).inverse();
    {SINV_TEST}


    //---------------------------------//
    //       KALMAN GAIN CHECK         //
    //---------------------------------//
    // Compute the Kalman Gain K = PHᵀ ⋅ S⁻¹
    {SPARSE_KALMAN_GAIN}

    // Compare with the dense computation
    const Eigen::MatrixXd K_d = (PHT_d * Sinv_d);
    {K_TEST}

    //---------------------------------//
    //       STATE UPDATE CHECK        //
    //---------------------------------//
    // Compute the Kalman Gain K = PHᵀ ⋅ S⁻¹
    {ErrorState} dx;
    {SPARSE_COMPUTE_ERROR_STATE}

    // Compare with the dense computation
    const Eigen::VectorXd dx_d = (PHT_d * Sinv_d) * res_d;
    {STATE_UPDATE_CHECK}
    EXPECT_FALSE(dx_d.isZero());
    MAT_EQ(dx_d, dx.dense({COVARIANCE_ARGS}));

    //---------------------------------//
    //     COVARIANCE UPDATE CHECK     //
    //---------------------------------//
    // Compute the Covariance Update
    // P⁺ = P - PHᵀ ⋅ S⁻¹ ⋅ HP
    //    = P - K ⋅ (PHᵀ)ᵀ
    {SPARSE_COVARIANCE_UPDATE}

    const Eigen::MatrixXd Pup_d = P_d - (K_d * PHT_d.transpose());
    EXPECT_FALSE(Pup_d.isZero());
    MAT_EQ(Pup_d, cov.dense({COVARIANCE_ARGS}));
}}
"""


def state_update_check(cfg):
    pass


def pht_test(cfg, key):
    P = create_symbolic_covariance(cfg, "cov")
    H = create_symbolic_measurement_jacobian(cfg, key)
    PHT = P * H.T
    return sparse_mat_eq(PHT, "PHT_d", "PHT", already_checked_sizes=[H[0, 0].rows])


def k_test(cfg, key):
    meas_cfg = cfg["measurements"][key]
    meas_state = State(key, meas_cfg["type"])
    P = create_symbolic_covariance(cfg, "cov")
    H = create_symbolic_measurement_jacobian(cfg, key)
    PHT = P * H.T
    Sinv = SymBlockMatrix([[
        ConstBlock("Sinv", dim(meas_state), dim(meas_state), meas_state.dx, meas_state.dx)
    ]])
    K = PHT * Sinv
    return sparse_mat_eq(K, "K_d", "K", already_checked_sizes=[H[0, 0].rows])


def replicate_r(cfg, key):
    meas_cfg = cfg["measurements"][key]
    s = State(key, meas_cfg["type"])
    if s.variable:
        return f".replicate({s.counter_var}, 1)"
    else:
        return ""


def residual_head(cfg, key):
    meas_cfg = cfg["measurements"][key]
    s = State(key, meas_cfg["type"])
    if s.variable:
        return f".head({size_var(s)})"
    else:
        return ""


def sparse_update_test_impl(cfg, key):
    states = cfg["state_composition"]
    meas_cfg = cfg["measurements"][key]
    meas_state = [State(key, meas_cfg["type"])]

    def impl_test():
        sparse_impl = sparse_meas_update(cfg, key)
        sparse_impl["SPARSE_KALMAN_GAIN"] = dedent(sparse_impl["SPARSE_KALMAN_GAIN"])
        sparse_impl["SPARSE_COVARIANCE_UPDATE"] = dedent(sparse_impl["SPARSE_COVARIANCE_UPDATE"])
        sparse_impl["SPARSE_COMPUTE_ERROR_STATE"] = dedent(
            sparse_impl["SPARSE_COMPUTE_ERROR_STATE"]
        )
        return SPARSE_UPDATE_TEST.format(
            **sparse_impl,
            Meas=f"{make_camel(key)}Meas",
            ErrorState=cfg["error_state_name"],
            SIZE_VARIABLES=size_variables(states + meas_state),
            BAIL_IF_VARIABLE_MEAS_EMPTY=bail_if_variable_meas_empty(cfg, key),
            MEAS_JAC_ARGS=dense_args(states + meas_state),
            COVARIANCE_ARGS=dense_args(states),
            PHT_TEST=pht_test(cfg, key),
            REPLICATE_R=replicate_r(cfg, key),
            RESIDUAL_HEAD=residual_head(cfg, key),
            SINV_TEST=sinv_test(meas_state[0]),
            K_TEST=k_test(cfg, key),
            STATE_UPDATE_CHECK=""
        )

    args = {f'int {s.counter_var}' for s in states + meas_state if s.variable}

    out = []
    if len(args) > 0:
        calling_args = {f'{s.counter_var}' for s in states + meas_state if s.variable}
        out.append(f"void do_{key}_update_test({', '.join(args)}) {{ ")
        out.append(impl_test())
        for n in ["max", "min", 'mid']:
            out.append(
                f"TEST({cfg['kalman_filter_class']}, MeasUpdate{make_camel(key, True)}{n.capitalize()}) {{"
            )
            out.append(num_var(n, states + meas_state))
            out.append(f"    do_{key}_update_test({', '.join(calling_args)});")
            out.append("}\n")
    else:
        out = []
        out.append(f"TEST({cfg['kalman_filter_class']}, MeasUpdate{make_camel(key, True)}) {{")
        out.append(impl_test())
    return "\n".join(out)


def sparse_update_test(cfg):
    out = []
    for key in cfg["measurements"].keys():
        out.append(sparse_update_test_impl(cfg, key))
    return "\n".join(out)


def boxplus_rules_test(cfg):
    ErrorState = cfg["error_state_name"]
    State = cfg["state_name"]
    states = cfg["state_composition"]

    def zero_add_test():
        out = [f"const {State} xp = x + zeros;"]
        for s in states:
            if s.variable:
                out.append(f"for (size_t i = 0; i < x.{s.counter_var}; ++i) {{")
                out.append(f"    {test_eq_compare(s.type)}(xp.{s.name}[i], x.{s.name}[i]);")
                out.append("}")
            else:
                out.append(f"{test_eq_compare(s.type)}(xp.{s.name}, x.{s.name});")
        return "\n        ".join(out)

    def add_subtract_test():
        out = [f"const {State} xp = x + (x2 - x);"]
        for s in states:
            if s.variable:
                out.append(f"for (size_t i = 0; i < x.{s.counter_var}; ++i) {{")
                out.append(f"    {test_eq_compare(s.type)}(xp.{s.name}[i], x2.{s.name}[i]);")
                out.append("}")
            else:
                out.append(f"{test_eq_compare(s.type)}(xp.{s.name}, x2.{s.name});")
        return "\n        ".join(out)

    def subtract_add_test():
        out = [f"const {ErrorState} dxp = (x + dx) - x;"]
        for s in states:
            if s.variable:
                out.append(
                    f"MAT_EQ(dxp.{s.name}_vec({State}::{s.MAX}), dx.{s.name}_vec({State}::{s.MAX}));"
                )
            else:
                out.append(f"MAT_EQ(dxp.{s.name}, dx.{s.name});")
        return "\n        ".join(out)

    def max_out_size():
        out = []
        for s in states:
            if s.variable:
                out.append(f"x.{s.counter_var} = {State}::{s.MAX};")
                out.append(f"x2.{s.counter_var} = {State}::{s.MAX};")
        return "\n    ".join(out)

    return {
        "State": State,
        "ErrorState": ErrorState,
        "ZERO_ADD_TEST": zero_add_test(),
        "ADD_SUBTRACT_TEST": add_subtract_test(),
        "SUBTRACT_ADD_TEST": subtract_add_test(),
        "MAX_OUT_SIZE": max_out_size(),
    }
