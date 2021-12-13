import numpy as np
from glob import glob


def print_first_pos(nodeid, x, y, z):
    return f"""$node_({nodeid}) set X_ {x}\n$node_({nodeid}) set Y_ {y}\n$node_({nodeid}) set Z_ {z}\n"""


def print_pos(nodeid, timestamp,  x, y, speed):
    return f"$ns_ at {timestamp} \"$node_({nodeid}) setdest {x} {y} {speed}\"\n"


def read_mobility(user_filename):
    return np.genfromtxt(user_filename, delimiter=" ")


def make_writeable(nodeid, user_array, num_samples=1000):
    out = ""

    if num_samples is not None:
        user_array = user_array[:num_samples]

    for index, sample in enumerate(user_array):
        if index == 0:
            out += print_first_pos(nodeid, sample[0], sample[1], 1)
        else:
            out += print_pos(nodeid, index, sample[0], sample[1], 1)
    return out


def gen_mobility():
    num_users = 100
    num_samples = 1000
    user_files = glob("san_francisco/*.txt")[:num_users]
    outfile = "output.tcl"

    # clear the contents of the file if pre-existing
    open(outfile, 'w').close()

    # loop to generate mobility for the given users
    for userid, u in enumerate(user_files):
        data = read_mobility(u)
        tcl_output = make_writeable(userid, data, num_samples=num_samples)

        with open(outfile, "a+") as f:
            f.write(tcl_output)


if __name__ == "__main__":
    gen_mobility()
