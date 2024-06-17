datapath=$1
mkdir $1
wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0002/2011_09_26_drive_0002_sync.zip -O $datapath/data.zip
unzip $datapath/data.zip -d $datapath/
rm $datapath/data.zip
