build:
	./gen_make.sh
	cd build
	make -j10
run:
	export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:SimOneAPI/Samples/lib/Linux64
	cd run
	vi config.ini
	./Trans
