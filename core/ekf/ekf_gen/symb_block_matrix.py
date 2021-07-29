import copy

from core.ekf.ekf_gen.symb_matrix import BaseBlock, ConstBlock, Add, Mult
from core.ekf.ekf_gen.symb_algebra import reduce


class SymBlockMatrix:
    def __init__(self, block_lists):
        self._block_list = block_lists
        self.rows = len(self._block_list)
        self.cols = len(self._block_list[0])

        # Check that sizes are right
        for r in self._block_list:
            num_rows = r[0].rows
            if len(r) != self.cols:
                raise RuntimeError("Mismatched block columns")
            for c in r:
                if c.rows != num_rows:
                    raise RuntimeError("Mismatched rows in block matrix")

        for c in range(self.cols):
            num_cols = self._block_list[0][c].cols
            for r in range(self.rows):
                block = self._block_list[r][c]
                if block.cols != num_cols:
                    raise RuntimeWarning("Mismatched cols in block matrix")

    def shape(self):
        return (self.rows, self.cols)

    def full_shape(self):
        def accumulate_expr(expr, dim):
            if isinstance(dim, str):
                return f"{expr} + {dim}"
            else:
                return expr + dim

        row_expr = 0
        for r in self._block_list:
            row_expr = accumulate_expr(row_expr, r[0].rows)
        col_expr = 0
        for c in self._block_list[0]:
            col_expr = accumulate_expr(col_expr, c.cols)
        return (row_expr, col_expr)

    def row(self, r):
        return self._block_list[r]

    def col(self, c):
        out = []
        for r in range(self.rows):
            out.append(self._block_list[r][c])
        return out

    def __getitem__(self, pos):
        r, c = pos
        return self._block_list[r][c]

    def add_impl(self, other, subtract=False):
        if self.rows != other.rows:
            raise RuntimeError("Mismatched block rows")
        if self.cols != other.cols:
            raise RuntimeError("Mismatched block cols")

        for c in range(self.cols):
            for r in range(self.rows):
                self_block = self[r, c]
                other_block = other[r, c]
                if self_block.rows != other_block.rows:
                    raise RuntimeWarning("Mismatched rows in block matrix")
                if self_block.cols != other_block.cols:
                    raise RuntimeWarning("Mismatched cols in block matrix")

        out_blocks = []
        for r in range(self.rows):
            row = []
            for c in range(self.cols):
                self_block = self._block_list[r][c]
                other_block = other._block_list[r][c]
                if not subtract:
                    row.append(self_block + other_block)
                else:
                    row.append(self_block - other_block)
            out_blocks.append(row)
        return SymBlockMatrix(out_blocks)

    def __add__(self, other):
        return self.add_impl(other, False)

    def __sub__(self, other):
        return self.add_impl(other, True)

    def scalar_multiply(self, other):
        out = []
        for r in range(self.rows):
            new_row = []
            for c in range(self.cols):
                new_row.append(other * self._block_list[r][c])
            out.append(new_row)
        return SymBlockMatrix(out)

    def __mul__(self, other):
        if isinstance(other, BaseBlock) and other.is_scalar:
            return self.scalar_multiply(other)

        if self.cols != other.rows:
            raise RuntimeError("Mismatched block rows")

        out = []
        for r in range(self.rows):
            new_row = []
            for c in range(other.cols):
                self_row = self.row(r)
                other_col = other.col(c)

                accum = ConstBlock(
                    "0", self_row[0].rows, other_col[0].cols, self_row[0].max_rows,
                    other_col[0].max_cols
                )
                for i, j in zip(self_row, other_col):
                    accum = accum + (i * j)
                new_row.append(accum)
            out.append(new_row)
        return SymBlockMatrix(out)

    def __repr__(self):
        out = []
        for r in range(self.rows):
            out.append("[" + ", ".join([item.name for item in self.row(r)]) + "]")
        return "[" + ",\n ".join(out) + "]"

    @property
    def T(self):
        out = []
        for c in range(self.cols):
            new_row = []
            for r in range(self.rows):
                new_row.append(self._block_list[r][c].T)
            out.append(new_row)
        return SymBlockMatrix(out)

    def reduce(self):
        for c in range(self.cols):
            for r in range(self.rows):
                self._block_list[r][c] = reduce(self._block_list[r][c])
        return self

    def gather_terms(self):
        def gather_expr(expr, path, collected):
            for i, t in enumerate(expr):
                if isinstance(t, Add) or isinstance(t, Mult):
                    path_to_term = path
                    path_to_term.append(i)
                    if t not in collected.keys():
                        collected[t] = {"path": path_to_term, "size": 1}
                    else:
                        collected[t]["path"].append(path_to_term)
                        collected[t]["size"] += 1

                    gather_expr(t, path_to_term, collected)

        collected = {}
        for r in range(self.rows):
            for c in range(self.cols):
                path = [r, c]
                gather_expr(self[r, c], path, collected)
        for k, v in collected.items():
            if v["size"] > 1:
                print(v["size"], k)

    def top_left_corner(self, r, c):
        total_rows = 0
        for r in range(0, r):
            total_rows += self[r, 0].rows

        total_cols = 0
        for c in range(0, c):
            total_cols += self[0, c].cols

        return total_rows, total_cols


def vstack(*args):
    if len(args) == 0:
        raise RuntimeError("Must supply arguments")

    for i, arg in enumerate(args):
        if i == 0:
            out = copy.deepcopy(arg._block_list)
        else:
            out.extend(arg._block_list)

    return SymBlockMatrix(out)


def hstack(args):
    if len(args) == 0:
        raise RuntimeError("Must supply arguments")

    for i, arg in enumerate(args):
        if i == 0:
            out = copy.deepcopy(arg._block_list)
        else:
            for r, _ in enumerate(out):
                out[r].extend(arg._block_list[r])

    return SymBlockMatrix(out)
