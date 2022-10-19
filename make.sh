set -ex

cd worldctr
cmake .
make
cd ..

cd remotectr
cmake .
make
cd ..

