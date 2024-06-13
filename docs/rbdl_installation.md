# RBDL Installation

1. Install the necessary dependencies:

```bash
sudo apt update && sudo apt install git-core cmake libeigen3-dev build-essential
```

2. Clone and build the project

```bash
git clone --recursive https://github.com/rbdl/rbdl
mkdir -p rbdl/build && cd rbdl/build
cmake -D CMAKE_BUILD_TYPE=Release ..
make
sudo make install
```

3. Add the RBDL library path to the environment variables

```bash
echo "export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH" >> ~/.bashrc
source ~/.bashrc
```

