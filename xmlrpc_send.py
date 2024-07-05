#!/usr/bin/env python
#
import xmlrpc.client as xml
import argparse

def main(args):
    parser = argparse.ArgumentParser()
    
    parser.add_argument("--xmlrpc", type=str, required=True,
        help="XMLRPC Full URL")
    parser.add_argument("--parameter", type=str, required=True,
        help="Parameter Name")
    parser.add_argument("--value", type=str, default=None,
        help="Parameter Value")
    
    args = parser.parse_args()
    
    rpc = xml.ServerProxy(args.xmlrpc)
    fcn = getattr(rpc,args.parameter)
    try:
        val = float(args.value)
    except:
        val = args.value
    if (val != None):
        ret = fcn(val)
    else:
        ret = fcn()
    print (ret)
    return 0

if __name__ == '__main__':
    import sys
    sys.exit(main(sys.argv))
