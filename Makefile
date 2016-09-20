all: debug release

release:
	mkdir -p Release
	cd Release;	cmake -DCMAKE_BUILD_TYPE=Release ..; make -j4; cp keepaway_player ../player/

debug:
	mkdir -p Debug
	cd Debug; cmake -DCMAKE_BUILD_TYPE=Debug ..; make -j4; cp keepaway_player ../player/

clean:
	cd Release; make clean
	cd Debug; make clean
	cd tools; make clean
	rm -fr hive*-Q* logs/* *.lock core core.* vgcore.* *.lock Q_H* console.log

