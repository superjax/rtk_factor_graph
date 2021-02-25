from core.ekf.ekf_gen.symb_matrix import ConstBlock, Mult, Add


def factor(expr):
    for i, t_i in enumerate(expr.terms):
        count_same_terms = [i]
        for j, t_j in enumerate(expr.terms):
            if i == j:
                continue
            if t_i == t_j:
                count_same_terms.append(j)
        if len(count_same_terms) > 1:
            expr.terms[i] = ConstBlock(f"{len(count_same_terms)}", 1, 1) * expr.terms[i]
            for j in reversed(count_same_terms):
                if j != i:
                    expr.terms.pop(j)
    return expr


def factor_scalars(expr):
    def strip_scalar(scalar, expr):
        for i, t_i in enumerate(expr):
            if t_i == scalar:
                if isinstance(t_i, Add):
                    raise RuntimeError("Cannot factor scalars out of Add blocks")
                if len(expr) == 1:
                    return expr, ConstBlock("I", t_i.rows, t_i.cols)
                elif len(expr) == 2:
                    return expr[i], expr[1 - i]
                else:
                    first = expr.terms.pop(i)
                    return first, expr

    def same_scalar_term(expr, other_expr):
        expr_scalar_terms = [t for t in expr if t.is_scalar]
        for t_i in other_expr:
            if t_i in expr_scalar_terms:
                return t_i
        return None

    while True:
        i = 0
        modified = False
        while i < len(expr):
            t_i = expr[i]
            j = i + 1
            while j < len(expr):
                t_j = expr[j]
                if not isinstance(t_i, Mult) or not isinstance(t_j, Mult):
                    j += 1
                    continue
                scalar = same_scalar_term(t_i, t_j)
                if scalar is not None:
                    first_i, rest_i = strip_scalar(scalar, t_i)
                    _, rest_j = strip_scalar(scalar, t_j)
                    expr.terms[i] = Mult(first_i, factor_scalars(Add(rest_i, rest_j)))
                    t_i = expr.terms[i]
                    expr.terms.pop(j)
                    if len(expr) == 1:
                        expr = expr.terms[0]
                    modified = True
                else:
                    j += 1
            i += 1
        if not modified:
            break
    return expr


def factor_first_term(expr):
    def strip_first_term(expr):
        if len(expr) == 1:
            return expr, ConstBlock("I", expr.cols, expr.cols)
        elif isinstance(expr, Mult):
            if len(expr) == 2:
                return expr[0], expr[1]
            else:
                first = expr.pop_front()
                return first, expr
        else:
            raise RuntimeError("Cannot factor anything besides ConstBlocks and Mults")

    def same_first_term(expr, other_expr):
        # find first non-scalar term
        expr_non_scalar_terms = [t for t in expr if not t.is_scalar]
        other_non_scalar_terms = [t for t in other_expr if not t.is_scalar]
        first_term = expr_non_scalar_terms[0]
        return first_term == other_non_scalar_terms[0]

    i = 0
    while i < len(expr):
        t_i = expr[i]
        j = i + 1
        while j < len(expr):
            t_j = expr[j]
            if isinstance(t_i, Add) or isinstance(t_j, Add):
                j += 1
                continue
            if same_first_term(t_i, t_j):
                first_i, rest_i = strip_first_term(t_i)
                _, rest_j = strip_first_term(t_j)
                expr.terms[i] = Mult(first_i, factor_first_term(Add(rest_i, rest_j)))
                t_i = expr[i]
                expr.terms.pop(j)
                if len(expr) == 1:
                    expr = expr[0]
            else:
                j += 1
        i += 1
        pass

    return expr


def factor_last_term(expr):
    def strip_last_term(expr):
        if len(expr) == 1:
            return ConstBlock("I", expr.rows, expr.rows), expr
        elif isinstance(expr, Mult):
            if len(expr) == 2:
                return expr[0], expr[1]
            else:
                last = expr.pop_back()
                return expr, last
        else:
            raise RuntimeError("Cannot factor anything besides ConstBlocks and Mults")

    def same_last_term(expr, other_expr):
        # find first non-scalar term
        expr_non_scalar_terms = [t for t in expr if not t.is_scalar]
        other_non_scalar_terms = [t for t in other_expr if not t.is_scalar]
        last_term = expr_non_scalar_terms[-1]
        return last_term == other_non_scalar_terms[-1]

    i = 0
    while i < len(expr):
        t_i = expr[i]
        j = i + 1
        while j < len(expr):
            t_j = expr[j]
            if isinstance(t_i, Add) or isinstance(t_j, Add):
                j += 1
                continue
            if same_last_term(t_i, t_j):
                rest_i, last_i = strip_last_term(t_i)
                rest_j, _ = strip_last_term(t_j)
                expr.terms[i] = Mult(Add(rest_i, rest_j), last_i)
                t_i = expr[i]
                expr.terms.pop(j)
                if len(expr) == 1:
                    expr = expr[0]
                    return expr
            else:
                j += 1
        i += 1
        pass

    return expr


def clean_up_negative(expr):
    pass


def combine_factors(expr):
    expr = factor_scalars(expr)
    expr = factor_first_term(expr)
    expr = factor_last_term(expr)
    return expr


def reduce(expr):
    if isinstance(expr, Add):
        expr = factor(expr)
        expr = combine_factors(expr)
        for i, t in enumerate(expr):
            if isinstance(t, Add):
                expr.terms[i] = reduce(t)

            if isinstance(t, Mult):
                for j, t_j in enumerate(t):
                    if isinstance(t_j, Add):
                        expr.terms[i].terms[j] = reduce(t_j)

    return expr
