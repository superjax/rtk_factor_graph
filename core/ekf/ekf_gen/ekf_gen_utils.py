import re


def accumulate_expr(expr, dim):
    if isinstance(dim, str):
        return f"{expr} + {dim}"
    else:
        return expr + dim


def get_matching_elements(items, condition):
    for item in items:
        if condition(item):
            return item
    raise RuntimeError("Unable to find matching item")


def begin_namespaces(cfg):
    return "\n".join([f"namespace {ns} {{" for ns in cfg['namespaces']])


def includes(cfg):
    if 'includes' not in cfg.keys():
        return ""
    else:
        return "\n".join([f'#include "{inc}' for inc in cfg['includes']])


def end_namespaces(cfg):
    return "\n".join(f"}} // end namespace {ns}" for ns in reversed(cfg['namespaces']))


def jac_member_name(s1, s2):
    if s1.name is not None:
        return f"d{make_camel(s1.name, True)}_d{make_camel(s2.name, True)}"
    else:
        return f"d{make_camel(s2.name, True)}"


def make_camel(snake, capitalize=False):
    words = snake.split('_')
    out = words[0]
    for i in range(1, len(words)):
        out += words[i].capitalize()
    if capitalize:
        out = out[0].upper() + out[1:]
    return out


def make_snake(camel):
    return re.sub(r'(?<!^)(?=[A-Z])', '_', camel).lower()


def state_size(state):
    if state == "DQuat":
        return 8
    elif state == "Quat":
        return 4
    elif state == "SO3":
        return 9
    elif state == "SE3":
        return 12
    elif state.startswith("Vec"):
        return int(re.match(r"Vec(\d+)", state).group(1))
    else:
        raise RuntimeError(f"Unknown state type {state}")


def error_state_size(state):
    if state == "DQuat":
        return 6
    elif state == "Quat":
        return 3
    elif state == "SO3":
        return 3
    elif state == "SE3":
        return 6
    elif state.startswith("Vec"):
        return int(re.match(r"Vec(\d+)", state).group(1))
    else:
        raise RuntimeError(f"Unknown state type {state}")


def is_state_manifold(state):
    if state == "DQuat":
        return True
    elif state == "Quat":
        return True
    elif state == "SO3":
        return True
    elif state == "SE3":
        return True
    elif state.startswith("Vec"):
        return False
    else:
        raise RuntimeError(f"Unknown state type {state}")


def test_eq_compare(state):
    if state == "DQuat":
        return "DQUAT_EQ"
    elif state == "Quat":
        return "QUAT_EQ"
    elif state == "SO3":
        return "SO3_EQUALS"
    elif state == "SE3":
        return "SE3_EQUALS"
    elif state.startswith("Vec"):
        return "MAT_EQ"
    else:
        raise RuntimeError(f"Unknown state type {state}")


def is_var(state):
    return state.startswith("Var")


def bundle_var(state):
    m = re.search(r"Var<([\w\d]*)", state)
    if m is None:
        raise RuntimeError(f"Unable to state bundle type from {state}")
    return m.group(1)


def bundle_size(state):
    m = re.search(r"Var<[\w\d]+,\s(\d+)>", state)
    if m is None:
        raise RuntimeError(f"Unable to state bundle type from {state}")
    return int(m.group(1))


class State:
    def __init__(self, name, kind):
        if name is not None:
            self.counter_var = f"num_{make_snake(name)}"
            self.MAX = f"MAX_{name.upper()}"
        else:
            self.counter_var = "num_obs"
        self.name = name
        if is_var(kind):
            state_type = bundle_var(kind)
            b_size = bundle_size(kind)
            base_size = state_size(state_type)
            base_error_size = error_state_size(state_type)
            self.variable = True
            self.x = b_size * base_size
            self.dx = b_size * base_error_size
            self.type = state_type
            self.manifold = is_state_manifold(state_type)
            self.bundle_size = b_size
            self.base_x = base_size
            self.base_dx = base_error_size
        else:
            self.variable = False
            self.x = state_size(kind)
            self.dx = error_state_size(kind)
            self.type = kind
            self.manifold = is_state_manifold(kind)

    def __eq__(self, other):
        return self.name == other.name and self.type == other.type

    def __repr__(self):
        return f"{self.name} ({self.dx})"
