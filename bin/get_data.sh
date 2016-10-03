DATA_DIR=$(pwd)/data
echo "using "$DATA_DIR" as data dir"
mkdir -p $DATA_DIR

wget --continue -O $DATA_DIR/dataset.bag.tar.gz http://bit.ly/udacity-dataset-2-1
unzip tar xvfz $DATA_DIR/dataset.bag.tar.gz
