import sys
import numpy as np
import pandas as pd

def parse_dh_param_file(dh_config_file):
    assert(dh_config_file is not None)
    f_line_contents = None
    with open(dh_config_file, "r") as f:
        f_line_contents = f.readlines()

    assert(f.closed)
    assert(f_line_contents is not None)
    # maybe not the most efficient/clean/etc. way to do this, but should only have to be done once so NBD
    dh_params = np.asarray([line.rstrip().split(',') for line in f_line_contents[1:]])
    dh_params = dh_params.astype(float)
    return dh_params


### TODO: parse a pox parameter file
    
def parse_pox_param_file(dh_config_file):
    # read in file at pre-defined path
    try:
        df = pd.read_csv(dh_config_file)
    except Exception as e:
        print("COULD NOT PARSE POX FILE!")
        print(e)
        return False, False
    
    # find M matrix
    m_mat = np.ones((4,4))
    for i in range(4):
        row = df.iloc[i][0:4]
        for id, key in enumerate(row.keys()):
            m_mat[i,id] = row[key]

    # find screw vectors
    s_lst = np.ones((5, 6))
    for i in range(5):
        row = df.iloc[i+5][0:6]
        for id, key in enumerate(row.keys()):
            s_lst[i, id] = row[key]

    return m_mat, s_lst
        
if __name__ == "__main__":
    m_mat, s_lst = parse_pox_param_file('./config/rx200_pox2.csv')
    print('M MATRIX:')
    print(m_mat)
    print('SCREW VECTORS:')
    print(s_lst)