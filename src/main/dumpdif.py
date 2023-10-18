#!/usr/bin/python
# -*- coding: UTF-8 -*-

import sys

def fix_value(value):
    value = value.replace('=', '')
    value = value.replace('\n', '')
    value = value.replace('\r', '')
    return value

if __name__ == "__main__":
    print("Fixar Param comparision")
    if len(sys.argv) != 3:
        print("wrong parameters")
        print("usage: python dumpdif.py FILE1 FILE2")
        sys.exit()
    
    print("key\t{}\t{}".format(sys.argv[1], sys.argv[2]))
    
    f1 = open( sys.argv[1] )
    f2 = open( sys.argv[2] )
    
    p1 = {}
    p2 = {}
    
    for l in f1:
        if( len(l) > 1 ):
            if l[0] != '#':
                params = l.split(' ')
                #print params
                
                if len(params) >= 3:
                    key = params[0] + ' ' + params[1]
                    value = ''
                    for i in range( 2, len(params) ):
                        value += params[i] + ' '
                        
                    p1[key] = fix_value(value) 
      
    
    for l in f2:
        if( len(l) > 1 ):
            if l[0] != '#':
                params = l.split(' ')
                #print params
                
                if len(params) >= 3:
                    key = params[0] + ' ' + params[1]
                    value = ''
                    for i in range( 2, len(params) ):
                        value += params[i] + ' '
                          
                        
                    p2[key] = fix_value(value) 
                                 
    
    for k, v in sorted(p1.items()):
        try:
            if p2[k] != v:
                print("{}:\t{}\t{}".format( k, v, p2[k]))
            p2.pop(k)
        except:
            print("{}:\t{}\t{}".format( k, v, 'NA'))
            
    
    for k, v in sorted(p2.items()):
        print("{}:\t{}\t{}".format( k, 'NA', v))
        
    #print p_diff
    