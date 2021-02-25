import copy


class BaseBlock:
    def __init__(self, rows, cols):
        self.rows = rows
        self.cols = cols
        self.is_scalar = (rows == 1 and cols == 1)
        self.is_zero = False
        self.is_identity = False
        self.is_negative = False

    def __add__(self, other):
        if (self.rows != other.rows) or (self.cols != other.cols):
            raise RuntimeError("Mismatched block sizes for + operator")
        elif self.is_zero:
            return other
        elif other.is_zero:
            return self

        return Add(self, other)

    def __sub__(self, other):
        tmp = copy.copy(other)
        tmp.set_negative()
        return self.__add__(tmp)

    def __mul__(self, other):
        if self.is_zero or other.is_zero:
            out_rows = self.rows if not self.is_scalar else other.rows
            out_cols = other.cols if not other.is_scalar else self.cols
            return ConstBlock("0", out_rows, out_cols)

        if self.is_identity and not other.is_scalar:
            return other
        elif other.is_identity and not self.is_scalar:
            return self

        return Mult(self, other)

    def set_negative(self):
        self.is_negative = True

    def __len__(self):
        assert False, "called len on base class"

    def __getitem__(self, i):
        assert False, "called getitem on base class"

    @property
    def shape(self):
        return (self.rows, self.cols)

    @property
    def name(self):
        return self.__repr__()

    def render(self):
        return self.name

    def __hash__(self):
        return hash(self.__repr__())

    @property
    def T(self):
        if self.is_zero:
            return ConstBlock("0", self.cols, self.rows)
        elif self.is_identity or self.is_scalar:
            return self
        else:
            return Transpose(self)


class ConstBlock(BaseBlock):
    def __init__(self, name, rows, cols):
        super().__init__(rows, cols)
        self._name = name
        self.is_identity = (name == "I")
        self.is_zero = (name == "0")
        if self.is_identity and rows != cols:
            raise ValueError("Identity Matrices must be square")

    def __repr__(self):
        return ("-" if self.is_negative else "") + self._name

    def render(self):
        if self.is_zero or self.is_identity:
            return ("-" if self.is_negative else "") + f"{self._name}_{self.rows}x{self.cols}"
        else:
            return ("-" if self.is_negative else "") + self._name

    def __hash__(self):
        return hash(self.__repr__())

    def __eq__(self, other):
        if self.rows != other.rows or self.cols != other.cols:
            return False
        return self.name == other.name

    def __getitem__(self, i):
        if i > 0:
            raise IndexError("ConstBlocks have only 1 term")
        return self

    def __len__(self):
        return 1


class Transpose(BaseBlock):
    def __init__(self, block):
        super().__init__(block.cols, block.rows)
        self.block = block

    def __repr__(self):
        if isinstance(self.block, ConstBlock):
            return f"{self.block.name}.T"
        else:
            return f"({self.block.name}).T"

    def render(self):
        if isinstance(self.block, ConstBlock):
            return f"{self.block.render()}.transpose()"
        else:
            return f"({self.block.render()}).transpose()"

    def __eq__(self, other):
        if self.rows != other.rows or self.cols != other.cols:
            return False
        return self.__repr__() == other.__repr__()

    def __hash__(self):
        return hash(self.__repr__())

    def __getitem__(self, i):
        return self

    def __len__(self):
        return 1


class Mult(BaseBlock):
    def __init__(self, a, b):
        if a.shape == (1, 1):
            rows = b.rows
            cols = b.cols
            self.commutative = True
        elif b.shape == (1, 1):
            rows = a.rows
            cols = a.cols
            self.commutative = True
        elif a.cols != b.rows:
            raise RuntimeError("Mismatched sizes for * operator")
        else:
            rows = a.rows
            cols = b.cols
            self.commutative = False
        super().__init__(rows, cols)

        self.terms = []
        if isinstance(a, Mult):
            self.terms.extend(a.terms)
        else:
            self.terms.append(a)

        if isinstance(b, Mult):
            self.terms.extend(b.terms)
        else:
            self.terms.append(b)

    def __hash__(self):
        return hash(self.__repr__())

    def __repr__(self):
        return ("-" if self.is_negative else "") + "*".join([
            f"({t.name})" if isinstance(t, Add) else t.name for t in self.terms
        ])

    def render(self):
        return ("-" if self.is_negative else "") + "*".join([
            f"({t.render()})" if isinstance(t, Add) else t.render() for t in self.terms
        ])

    def __eq__(self, other):
        if self.commutative:
            self_c = [t for t in self if t.is_scalar]
            other_c = [t for t in other if t.is_scalar]
            self_nc = [t for t in self if not t.is_scalar]
            other_nc = [t for t in other if not t.is_scalar]

            return ((len(self_c) == len(other_c)) and all([t in other_c for t in self_c])
                    and all([t in self_c for t in other_c]) and self_nc == other_nc)
        else:
            if not isinstance(other, Mult):
                return False
            return self.terms == other.terms

    def __getitem__(self, i):
        return self.terms[i]

    def __len__(self):
        return len(self.terms)

    def pop_front(self):
        for i, t in enumerate(self.terms):
            if t.is_scalar:
                continue
            popped_term = self.terms.pop(i)
            self.rows = popped_term.cols
            return popped_term

    def pop_back(self):
        for i in reversed(range(len(self.terms))):
            if self.terms[i].is_scalar:
                continue
            popped_term = self.terms.pop(i)
            self.cols = popped_term.rows
            return popped_term


class Add(BaseBlock):
    def __init__(self, a, b):
        if (a.rows != b.rows) or (a.cols != b.cols):
            raise RuntimeError("Mismatched block sizes for + operator")
        super().__init__(a.rows, a.cols)

        self.terms = []
        if isinstance(a, Add):
            self.terms.extend(a.terms)
        else:
            self.terms.append(a)

        if isinstance(b, Add):
            self.terms.extend(b.terms)
        else:
            self.terms.append(b)

    @property
    def shape(self):
        return (self.rows, self.cols)

    def __repr__(self):
        return (" - " if self.is_negative else " + ").join([t.name for t in self.terms])

    def render(self):
        return ("-" if self.is_negative else " + ").join([t.render() for t in self.terms])

    def __eq__(self, other):
        if not isinstance(other, Add):
            return False
        for t in other:
            if t not in self.terms:
                return False
        for t in self.terms:
            if t not in other.terms:
                return False
        return True

    def __hash__(self):
        return hash(self.__repr__())

    def __getitem__(self, i):
        return self.terms[i]

    def __len__(self):
        return len(self.terms)
