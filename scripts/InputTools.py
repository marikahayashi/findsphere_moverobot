# -*- coding: utf-8 -*-
# This code is distributed under the 3-Clause BSD license (New BSD license).
# 基本的に作者の名前を書いていただければ、商用利用も可能です。なお、保証はいたしかねます。

__author__ = 'alfredplpl'

# This fucntion can use on Unix systems (Mac or  Linux) .
TIMEOUT_VAL=-1
def waitKey(miliseconds):

    import signal

    # a function to raise a exception.
    def timeout(signum, frame): raise RuntimeError("timeout.")

    signal.signal(signal.SIGALRM,timeout)
    signal.alarm(miliseconds/1000)
    try:
        key=raw_input()
    except(RuntimeError):
        key=TIMEOUT_VAL
    finally:
        signal.alarm(0)

    return key

#A sample code
#The following code doesn't run if you import this.
#importしても実行されないので無視してimportしてください
if __name__ == "__main__":

    print "Input any key..."
    key=waitKey(0)
    key=TIMEOUT_VAL
    while(key==TIMEOUT_VAL):
        print "wait for a key..."
        #wait for 1000msec
        key=waitKey(1000)
        print key

# References: http://stackoverflow.com/questions/1335507/keyboard-input-with-timeout-in-python
# ライセンスに関する参考URL: http://osdn.jp/projects/opensource/wiki/licenses%2Fnew_BSD_license

# Copyright (c) 2015, alfredplpl
# All rights reserved.
