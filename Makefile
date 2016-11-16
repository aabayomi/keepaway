all: debug release
	cd player; ln -sf ../Release/keepaway_player .

release: tools
	mkdir -p Release
	cd Release;	cmake -DCMAKE_BUILD_TYPE=Release ..; make -j `nproc`
	cd player; ln -sf ../Release/keepaway_player .

debug: tools
	mkdir -p Debug
	cd Debug; cmake -DCMAKE_BUILD_TYPE=Debug ..; make -j `nproc`
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

cleanall:
	cd tools; make clean
	rm -f /run/lock/*.file_lock
	rm -f /run/shm/*.shm
	rm -fr /run/shm/sem.*
	rm -fr Debug Release
	rm -fr keeper_Q*.gz taker_Q*.gz logs/* *.lock core core.* vgcore.* 
	rm -fr *.lock console.log nohup.out *.dot *.xml
	./kill.sh 
	killall -q keepaway_player 1>/dev/null 2>&1
