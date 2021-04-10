import os
import pathlib
import re


def get_keys():
    log_keys_path = os.path.join(
        pathlib.Path(__file__).parent.absolute(), "../common/logging/log_key.h"
    )

    f = open(log_keys_path)
    text = f.read()
    print(text)

    list_of_keys = re.search(r"enum LogKey.\{(.*?)\};", text, re.DOTALL).group(1)
    print(list_of_keys)

    keys = re.findall(r"([A-Z_0-9]+),?", list_of_keys)

    out = {}
    for i, key in enumerate(keys):
        out[key] = i

    return out


LOG_KEYS = get_keys()

print(LOG_KEYS)
