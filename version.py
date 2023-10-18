#!/usr/bin/python
# -*- coding: UTF-8 -*-

import sys



import json
import os

def get_ver():
    with open('ver.json') as config_file:
        data = json.load(config_file)

    version = data['version']
    sub_version = data['sub_version']

    return (version, int(sub_version))
    
def get_ver_str(version, sub_version):
    return "{}.{:03}".format(version, sub_version)
    
def get_version_string():
    v, s = get_ver()
    return get_ver_str( v, s )

    
def set_ver( v, sv ):
    data = {
        "version" : v,
        "sub_version" : sv
        }
    
    with open('ver.json', 'w') as outfile:
        json.dump(data, outfile)
    print('Updating src/main/ver.h...')
    with open('src/main/ver.h', 'wt') as outfile:
        s = '#pragma once\n#define MY_VERSION (2)\n#define MY_SUBVERSION ({})\n'.format(sv)
        outfile.write( s )
        outfile.close()
        
   
def git_commit( comment = 'empty' ):
    v, sv = get_ver()
    
    #sv += 1
    
    ver_str = get_ver_str( v, sv )
    
    print("Version now is [{}]".format(ver_str))
    
    
    
    os.system('git add src/main/ver.h')
    
    rename_cmd = 'mv obj/inav_1.9.1_MATEKF405.hex obj/cxnav_{}_{}.hex'.format(v, sv)
    os.system(rename_cmd)
    
    cmd_string = 'git commit -m "{} -> {}"'.format( ver_str, comment )
    
    print(cmd_string)
    
    os.system( cmd_string )
    
    print('git_push')
    
    os.system( 'git push' )
    
    sv += 1
    set_ver( v, sv )
    
    
    
if __name__ == "__main__":
    if len(sys.argv) == 2:
        git_commit( sys.argv[1] )
    else:
        print("Error! len = {} \nUsage:\nversion.py [git comment]".format(len(sys.argv)))