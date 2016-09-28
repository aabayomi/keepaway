all: debug release
	cd player; ln -sf ../Release/keepaway_player .

release: tools
	mkdir -p Release
	cd Release;	cmake -DCMAKE_BUILD_TYPE=Release ..; make -j4
	cd player; ln -sf ../Release/keepaway_player .

debug: tools
	mkdir -p Debug
	cd Debug; cmake -DCMAKE_BUILD_TYPE=Debug ..; make -j4
	cd player; ln -sf ../Debug/keepaway_player .

clean:
	cd tools; make clean
	if [ -d Release ]; then \
		cd Release; make clean; \
	fi
	if [ -d Debug ]; then \
		cd Debug; make clean; \
	fi

tools:
	cd tools; make

cleanall: clean
	rm -fr Debug Release
	rm -fr hive*-Q* logs/* *.lock core core.* vgcore.* *.lock Q_H* console.log nohup.out
