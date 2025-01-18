#!/bin/bash
cd build
cmake -DTILIQUA_HW_MAJOR=2 .. && make -j8 && cp dirtyJtag.uf2 apfbug-r2.uf2
cmake -DTILIQUA_HW_MAJOR=3 .. && make -j8 && cp dirtyJtag.uf2 apfbug-r3.uf2
cmake -DTILIQUA_HW_MAJOR=4 .. && make -j8 && cp dirtyJtag.uf2 apfbug-r4.uf2
