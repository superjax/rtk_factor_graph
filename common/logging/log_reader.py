#!/usr/bin/python3

import yaml
import numpy as np
import numpy.lib.recfunctions
import os
import re

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
        data = yaml.load(header_file.read(), Loader=yaml.FullLoader)

    dtype = np.dtype(make_dtype(data["format"]))

    return dtype


def get_log_files(file_path):
    if not re.search(r'\d{8}_\d{6}\/?$', file_path):
        raise RuntimeError(f"Incorrectly formatted log directory {file_path}")

    manifest = yaml.load(
        open(os.path.join(file_path, 'manifest.yml')).read(), Loader=yaml.FullLoader
    )

    files = {
        key: os.path.join(file_path, name)
        for key, name in zip(manifest["keys"], manifest["key_names"])
    }

    # load amended topics
    if "amends" in manifest:
        amendment_files = get_log_files(manifest["amends"])
        amendment_files.update(files)
        files = amendment_files

    return files


def load(file_path, streams=None, normalize_time=True):
    # Load the most recent log if not supplied a direct id
    if not re.search(r'\d{8}_\d{6}\/?$', file_path):
        file_path = os.path.join(file_path, sorted(os.listdir(file_path))[-1])

    files = get_log_files(file_path)

    log = {}

    for k, stream in files.items():
        log_path = stream + ".log"
        hdr_path = stream + ".yml"

        print("loading {}".format(log_path))
        dtype = load_header(hdr_path)
        data = np.fromfile(log_path, dtype=dtype)

        if normalize_time and 't' in data.dtype.fields:
            print("normalizing_time")
            data = numpy.lib.recfunctions.rename_fields(data, {"t": "stamp"})
            data = numpy.lib.recfunctions.append_fields(
                data,
                't',
                data=data['stamp']['sec'].astype(np.float64) +
                data['stamp']['nsec'].astype(np.float64) / 1e9
            )
            data = data.data
        log[k] = data
    return log


if __name__ == "__main__":
    data = load("data/glonass.log")
    print(data)
