# gcs — C core, native (for the Python reference tests) and WebAssembly (for the web app).
#
#   make            native shared library in build/
#   make wasm       web/src/wasm/gcs.{js,wasm}   (needs emsdk: source ~/emsdk/emsdk_env.sh)
#   make test       Python test suite, C core tests included

UNAME := $(shell uname -s)
EXT := $(if $(filter Darwin,$(UNAME)),.dylib,.so)
SRC := $(wildcard csrc/*.c)
CFLAGS := -O2 -Wall -Wextra -std=c11

EXPORTS := _gcs_kernel_count,_gcs_kernel_info,\
_gcs_min_norm_lstsq,_gcs_rrqr,_gcs_svd,_gcs_rank_nullspace,_gcs_lu_solve,\
_gcs_system_new,_gcs_system_free,_gcs_system_n_res,_gcs_system_n_free,_gcs_system_nnz,\
_gcs_system_set_x,_gcs_system_get_x,_gcs_system_get_z,_gcs_system_full_x,\
_gcs_system_set_consts,_gcs_system_set_all_consts,\
_gcs_system_residuals,_gcs_system_jacobian_dense,\
_gcs_system_csr_indptr,_gcs_system_csr_indices,_gcs_system_csr_data,\
_gcs_system_hard,_gcs_system_max_hard_residual,_gcs_system_constraint_errors,\
_gcs_system_rank,_gcs_system_solve,_malloc,_free

.PHONY: all wasm test clean

all: build/libgcs$(EXT)

build/libgcs$(EXT): $(SRC) csrc/gcs.h csrc/system.h csrc/sparse.h
	@mkdir -p build
	cc $(CFLAGS) -shared -fPIC -o $@ $(SRC) -lm

wasm: web/src/wasm/gcs.js

web/src/wasm/gcs.js: $(SRC) csrc/gcs.h csrc/system.h csrc/sparse.h
	@mkdir -p web/src/wasm
	emcc -O3 -std=c11 $(SRC) -o $@ \
	  -sMODULARIZE=1 -sEXPORT_ES6=1 -sENVIRONMENT=web,worker,node \
	  -sALLOW_MEMORY_GROWTH=1 -sINITIAL_MEMORY=33554432 \
	  -sEXPORTED_FUNCTIONS='$(EXPORTS)' \
	  -sEXPORTED_RUNTIME_METHODS='["HEAPF64","HEAPU8","HEAP32","HEAPU32"]' \
	  -sSTACK_SIZE=1048576 --no-entry

test: all
	.venv/bin/pytest -q

clean:
	rm -rf build web/src/wasm/gcs.js web/src/wasm/gcs.wasm
