from ultralytics import YOLO


def train():
    model = YOLO('yolo11n.pt')

    model.train(
        data='data_local.yaml',
        epochs=100,
        imgsz=320,
        device=0,
        project='speedy_vision',
        name='yolo11n_speedy_rpi'
    )

    print("\n--- Treinamento Concluído ---")

    print("Exportando para NCNN (Alta performance ARM/RPi)...")
    model.export(format='ncnn', imgsz=320)

    print("\n--- Processo Finalizado ---")
    print("Os modelos otimizados estão na pasta 'yolo/speedy_vision/yolo11n_speedy_rpi/weights/'")


if __name__ == '__main__':
    train()
