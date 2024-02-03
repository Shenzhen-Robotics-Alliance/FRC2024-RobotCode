python3 train_ssd.py --dataset-type=voc --data=data/NoteDetection/ --model-dir=models/Not
eDetection --batch-size=12 --epochs=30 --workers=16;
# run under linux subsystem for windows, after installing nvidia drivers, cuda-accelerated pytorch and requirements.txt
# for more information, please check: https://github.com/dusty-nv/jetson-inference/blob/master/docs/pytorch-collect-detection.md