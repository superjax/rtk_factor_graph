#!/usr/bin/python3

import json
import numpy as np
import numpy.lib.recfunctions
import os

DATATYPES = {
    0: np.float64,
    1: np.float32,
    2: np.int64,
    3: np.int32,
    4: np.int16,
    5: np.int8,
    6: np.uint64,
    7: np.uint32,
    8: np.uint16,
    9: np.uint8,
}


def make_dtype(format):
    if "cols" in format.keys():
        return (DATATYPES[format["type"]], (format["rows"], format["cols"]))
    elif "rows" in format.keys():
        return (DATATYPES[format["type"]], format["rows"])
    elif "type" in format.keys():
        return DATATYPES[format["type"]]

    if (len(format) == 1):
        return (make_dtype(next(iter(format.values()))))

    dtype = []
    for item in format.items():
        dtype.append((item[0], make_dtype(item[1])))
    return dtype


def load_header(hdr_path):
    with open(hdr_path) as header_file:
        data = json.loads(header_file.read())

    dtype = np.dtype(make_dtype(data["format"]))

    return dtype


def load(file_path, normalize_time=True):
    root_path = os.path.splitext(file_path)[0]
    log_path = root_path + ".log"
    hdr_path = root_path + ".hdr"

    print("loading {}".format(file_path))
    dtype = load_header(hdr_path)
    data = np.fromfile(log_path, dtype=dtype)

    if normalize_time and 't' in data.dtype.fields:
        print("normalizing_time")
        data = numpy.lib.recfunctions.rename_fields(data, {"t": "stamp"})
        data = numpy.lib.recfunctions.append_fields(
            data,
            't',
            data=data['stamp']['sec'].astype(np.float64) +
            data['stamp']['nsec'].astype(np.float64) / 1e9)

    return data


if __name__ == "__main__":
    data = load("data/glonass.log")
    print(data)
