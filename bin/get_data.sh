DATA_DIR=$(pwd)/data
echo "using "$DATA_DIR" as data dir"

mkdir -p DATA_DIR
cd DATA_DIR

wget --continue https://archive.org/download/comma-dataset/comma-dataset.zip
unzip tar xvfz somefilename.tar.gz
