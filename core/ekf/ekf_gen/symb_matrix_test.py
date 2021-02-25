import unittest

from core.ekf.ekf_gen.symb_matrix import ConstBlock
from core.ekf.ekf_gen.symb_block_matrix import SymBlockMatrix
from core.ekf.ekf_gen.symb_algebra import factor_first_term, factor_scalars, factor_last_term, reduce    # noqa


class TestSymbBlock(unittest.TestCase):
    def test_mult(self):
        A = ConstBlock("A", 3, 3)
        B = ConstBlock("B", 3, 4)

        C = A * B

        self.assertEqual(C.rows, 3)
        self.assertEqual(C.cols, 4)
        self.assertEqual(C.name, "A*B")

        D = ConstBlock("D", 4, 23)
        E = C * D

        self.assertEqual(E.rows, 3)
        self.assertEqual(E.cols, 23)
        self.assertEqual(E.name, "A*B*D")

        dt = ConstBlock("dt", 1, 1)
        F = E * dt
        self.assertEqual(F.rows, 3)
        self.assertEqual(F.cols, 23)
        self.assertEqual(F.name, "A*B*D*dt")

        G = F * dt
        self.assertEqual(G.rows, 3)
        self.assertEqual(G.cols, 23)
        self.assertEqual(G.name, "A*B*D*dt*dt")

    def test_add(self):
        A = ConstBlock("A", 3, 3)
        B = ConstBlock("B", 3, 3)

        C = A + B

        self.assertEqual(C.rows, 3)
        self.assertEqual(C.cols, 3)
        self.assertEqual(C.name, "A + B")

        D = ConstBlock("D", 3, 12) * ConstBlock("E", 12, 3)
        E = C + D

        self.assertEqual(E.rows, 3)
        self.assertEqual(E.cols, 3)
        self.assertEqual(E.name, "A + B + D*E")

    def test_transpose(self):
        A = ConstBlock("A", 3, 3)
        B = ConstBlock("B", 3, 4)

        C = A * B
        D = C.T

        self.assertEqual(D.cols, 3)
        self.assertEqual(D.rows, 4)
        self.assertEqual(D.name, "(A*B).T")


class TestSymBlockMatrix(unittest.TestCase):
    def test_init(self):
        # yapf: disable
        A = SymBlockMatrix(
            [[ConstBlock("I", 3, 3), ConstBlock("b", 3, 1), ConstBlock("c", 3, 15)],
                [ConstBlock("d", 9, 3), ConstBlock("0", 9, 1), ConstBlock("e", 9, 15)],
                [ConstBlock("f", 4, 3), ConstBlock("0", 4, 1), ConstBlock("h", 4, 15)],
                [ConstBlock("g", 2, 3), ConstBlock("0", 2, 1), ConstBlock("i", 2, 15)]]
        )
        # yapf: enable

        self.assertEqual(A.rows, 4)
        self.assertEqual(A.cols, 3)
        self.assertEqual(A[1, 1].name, '0')
        self.assertEqual(A[1, 1].rows, 9)
        self.assertEqual(A[1, 1].cols, 1)
        self.assertEqual(A[0, 2].name, 'c')
        self.assertEqual(A[0, 2].rows, 3)
        self.assertEqual(A[0, 2].cols, 15)

    def test_add(self):
        # yapf: disable
        A = SymBlockMatrix(
            [[ConstBlock("a", 3, 3), ConstBlock("b", 3, 1), ConstBlock("c", 3, 15)],
                [ConstBlock("d", 9, 3), ConstBlock("0", 9, 1), ConstBlock("e", 9, 15)],
                [ConstBlock("f", 4, 3), ConstBlock("0", 4, 1), ConstBlock("n", 4, 15)],
                [ConstBlock("g", 15, 3), ConstBlock("0", 15, 1), ConstBlock("I", 15, 15)]]
        )
        B = SymBlockMatrix(
            [[ConstBlock("h", 3, 3), ConstBlock("i", 3, 1), ConstBlock("0", 3, 15)],
                [ConstBlock("j", 9, 3), ConstBlock("0", 9, 1), ConstBlock("e", 9, 15)],
                [ConstBlock("k", 4, 3), ConstBlock("m", 4, 1), ConstBlock("o", 4, 15)],
                [ConstBlock("l", 15, 3), ConstBlock("0", 15, 1), ConstBlock("I", 15, 15)]]
        )
        # yapf: enable

        C = A + B
        self.assertEqual(C[0, 0].name, "a + h")
        self.assertEqual(C[1, 0].name, "d + j")
        self.assertEqual(C[2, 0].name, "f + k")
        self.assertEqual(C[3, 0].name, "g + l")
        self.assertEqual(C[0, 1].name, "b + i")
        self.assertEqual(C[1, 1].name, "0")
        self.assertEqual(C[2, 1].name, "m")
        self.assertEqual(C[3, 1].name, "0")
        self.assertEqual(C[0, 2].name, "c")
        self.assertEqual(C[1, 2].name, "e + e")
        self.assertEqual(C[2, 2].name, "n + o")
        self.assertEqual(C[3, 2].name, "I + I")

    def test_mul(self):
        # yapf: disable
        A = SymBlockMatrix(
            [[ConstBlock("a", 3, 3), ConstBlock("b", 3, 1), ConstBlock("c", 3, 7)],
                [ConstBlock("d", 9, 3), ConstBlock("0", 9, 1), ConstBlock("e", 9, 7)],
                [ConstBlock("f", 4, 3), ConstBlock("0", 4, 1), ConstBlock("q", 4, 7)],
                [ConstBlock("g", 7, 3), ConstBlock("0", 7, 1), ConstBlock("I", 7, 7)]]
        )
        B = SymBlockMatrix(
            [[ConstBlock("h", 3, 6), ConstBlock("I", 3, 3), ConstBlock("0", 3, 7), ConstBlock("0", 3, 4)],
                [ConstBlock("j", 1, 6), ConstBlock("0", 1, 3), ConstBlock("m", 1, 7), ConstBlock("o", 1, 4)],
                [ConstBlock("k", 7, 6), ConstBlock("n", 7, 3), ConstBlock("I", 7, 7),  ConstBlock("p", 7, 4)]]
        )
        # yapf: enable

        C = A * B

        self.assertEqual(C.rows, 4)
        self.assertEqual(C.cols, 4)
        self.assertEqual(C[0, 0].name, "a*h + b*j + c*k")
        self.assertEqual(C[1, 0].name, "d*h + e*k")
        self.assertEqual(C[2, 0].name, "f*h + q*k")
        self.assertEqual(C[3, 0].name, "g*h + k")
        self.assertEqual(C[0, 1].name, "a + c*n")
        self.assertEqual(C[1, 1].name, "d + e*n")
        self.assertEqual(C[2, 1].name, "f + q*n")
        self.assertEqual(C[3, 1].name, "g + n")
        self.assertEqual(C[0, 2].name, "b*m + c")
        self.assertEqual(C[1, 2].name, "e")
        self.assertEqual(C[2, 2].name, "q")
        self.assertEqual(C[3, 2].name, "I")
        self.assertEqual(C[0, 3].name, "b*o + c*p")
        self.assertEqual(C[1, 3].name, "e*p")
        self.assertEqual(C[2, 3].name, "q*p")
        self.assertEqual(C[3, 3].name, "p")

    def test_block_transpose(self):
        # yapf: disable
        A = SymBlockMatrix(
            [[ConstBlock("a", 3, 3), ConstBlock("b", 3, 1), ConstBlock("c", 3, 7)],
                [ConstBlock("d", 9, 3), ConstBlock("0", 9, 1), ConstBlock("e", 9, 7)],
                [ConstBlock("f", 4, 3), ConstBlock("0", 4, 1), ConstBlock("q", 4, 7)],
                [ConstBlock("g", 7, 3), ConstBlock("0", 7, 1), ConstBlock("I", 7, 7)]]
        )
        # yapf: enable

        B = A.T

        self.assertEqual(B.rows, 3)
        self.assertEqual(B.cols, 4)

        self.assertEqual(B[0, 0].shape, (3, 3))
        self.assertEqual(B[0, 0].name, 'a.T')
        self.assertEqual(B[1, 0].shape, (1, 3))
        self.assertEqual(B[1, 0].name, 'b.T')
        self.assertEqual(B[0, 1].shape, (3, 9))
        self.assertEqual(B[0, 1].name, 'd.T')

    def test_scalar_multiply(self):
        # yapf: disable
        A = SymBlockMatrix(
            [[ConstBlock("a", 3, 3), ConstBlock("b", 3, 1), ConstBlock("c", 3, 7)],
             [ConstBlock("d", 9, 3), ConstBlock("0", 9, 1), ConstBlock("e", 9, 7)],
             [ConstBlock("f", 4, 3), ConstBlock("0", 4, 1), ConstBlock("q", 4, 7)],
             [ConstBlock("g", 7, 3), ConstBlock("0", 7, 1), ConstBlock("I", 7, 7)]]
        )
        # yapf: enable

        dt = ConstBlock("dt", 1, 1)

        Adt = A * dt

        # yapf: disable
        B = SymBlockMatrix(
            [[ConstBlock("a", 3, 3), ConstBlock("b", 3, 6), ConstBlock("c", 3, 4)],
             [ConstBlock("d", 6, 3), ConstBlock("0", 6, 6), ConstBlock("e", 6, 4)],
             [ConstBlock("f", 4, 3), ConstBlock("0", 4, 6), ConstBlock("q", 4, 4)]]
        )
        # yapf: disable

        BBdt2 = B*B*dt*dt

        self.assertEqual(str(BBdt2),
                         "[[dt*dt*(a*a + b*d + c*f), dt*dt*a*b, dt*dt*(a*c + b*e + c*q)],\n" +
                         " [dt*dt*(d*a + e*f), dt*dt*d*b, dt*dt*(d*c + e*q)],\n" +
                         " [dt*dt*(f*a + q*f), dt*dt*f*b, dt*dt*(f*c + q*q)]]")


class TestSimplify(unittest.TestCase):
    def test_reduce_factor(self):
        A = ConstBlock("A", 3, 3)
        B = ConstBlock("B", 3, 3)
        C = ConstBlock("C", 3, 3)

        expr = B+A+B+B+C+C+C+A+C+A+B+C

        red = reduce(expr)

        self.assertEquals(red.name, "4*B + 3*A + 5*C")

    def test_reduce_factor(self):
        A = ConstBlock("A", 3, 3)
        B = ConstBlock("B", 3, 3)
        c = ConstBlock("c", 1, 1)
        d = ConstBlock("d", 1, 1)
        e = ConstBlock("e", 1, 1)
        f = ConstBlock("f", 1, 1)

        expr = A*c*d*B + A*d*c + B*d*f*c*e

        red = factor_scalars(expr)

        self.assertEqual(red.name, "d*c*(A*B + A + B*f*e)")

    def test_factor_first(self):
        A = ConstBlock("A", 3, 3)
        B = ConstBlock("B", 3, 3)
        C = ConstBlock("C", 3, 3)
        D = ConstBlock("D", 3, 3)
        E = ConstBlock("E", 3, 3)

        # A*B*(C*D*E + A*B*C + I) + B*C*(C*D+A*D)
        expr = A*B*C*D*E + A*B*A*B*C + A*B + B*C*C*D + B*C*A*D
        red = factor_first_term(expr)

        self.assertEqual(red.name, "A*B*(C*D*E + A*B*C + I) + B*C*(C*D + A*D)")

    def test_factor_last(self):
        A = ConstBlock("A", 3, 3)
        B = ConstBlock("B", 3, 3)
        C = ConstBlock("C", 3, 3)
        D = ConstBlock("D", 3, 3)
        E = ConstBlock("E", 3, 3)

        # (C*D*E + A*B*C + I)*A + (C*D+A*D)*B
        expr = C*D*E*A + A*C*A + A + C*D*B + A*D*B
        red = factor_last_term(expr)

        self.assertEqual(red.name, "(C*D*E + A*C + I)*A + (C*D + A*D)*B")

    def test_reduce(self):
        A = ConstBlock("A", 3, 3)
        B = ConstBlock("B", 3, 3)
        C = ConstBlock("C", 3, 3)
        t = ConstBlock("t", 1, 1)
        d = ConstBlock("d", 1, 1)

        # 2*((A + B + C)*A) + 3*d*t*C*(A + I)*B + 6*A*C
        expr = A*A + B*A + C*A + A*A + B*A + C*A + d*t*C*A*B + d*t*C*B + A*C+A*C+A*C+A*C+A*C+A*C \
            + d*t*C*A*B + d*t*C*A*B + d*t*C*B + d*t*C*B
        red = reduce(expr)

        self.assertEqual(red.name, "2*(A + B + C)*A + 3*d*t*C*(A + I)*B + 6*A*C")



if __name__ == '__main__':
    unittest.main()
