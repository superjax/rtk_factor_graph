import re


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


def is_lie(state):
    return not state.startswith("Vec")


def get_size(state):
    if state == "DQuat":
        return 8, 6
    elif state == "Quat":
        return 4, 3
    elif state == "SO3":
        return 9, 3
    elif state == "SE3":
        return 12, 6
    elif state.startswith("Vec"):
        size = int(re.match(r"Vec(\d+)", state).group(1))
        return size, size
    else:
        raise RuntimeError(f"Unknown state type {state}")
