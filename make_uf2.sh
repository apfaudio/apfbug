#!/bin/bash
cd build
cmake -DTILIQUA_HW_MAJOR=5 .. && make -j8 && cp dirtyJtag.uf2 apfbug-r5.uf2
