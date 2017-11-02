echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz

echo "Converting vocabulary to binary version"
./bin_vocabulary
cd ..
