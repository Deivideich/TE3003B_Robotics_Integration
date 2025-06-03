from flask import Flask, render_template, request
import requests
import json
import schedule
import time
import threading
import os
import base64

app = Flask(__name__)
result = None
result_img = None

UPLOAD_FOLDER = 'uploads'
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

def call_api():
    global result, result_img
    url_trailer = 'http://127.0.0.1:8042/gettrailersdata' #TODO
    data_trailer = {}
    headers_trailer = {'Content-type': 'application/json'}

    response_trailer = requests.post(url_trailer, data=json.dumps(data_trailer), headers=headers_trailer)
    result = response_trailer.json()
    with open('result.json', 'w') as f:
        json.dump(result, f)

    url_img = 'http://127.0.0.1:8042/getimage' #TODO
    data_img = {}
    headers_img = {'Content-type': 'application/json'}
    response_img = requests.post(url_img, data=json.dumps(data_img), headers=headers_img)

    result_img = response_img.json()
    with open('result_img.json', 'w') as f:
        json.dump(result_img, f)

def schedule_api_call():
    while True:
        schedule.run_pending()
        time.sleep(1)

if __name__ == '__main__':
    call_api()  # Initial call
    schedule.every(0.5).seconds.do(call_api)  # Schedule call every half second

    t = threading.Thread(target=schedule_api_call)
    t.start()

    @app.route('/api/trailers')
    def get_result():
        print('result', result)
        return {
            'trailers': [
                {'id': int(result['ids'][0]), 'boxes': int(result['boxes'][0])},
                {'id': int(result['ids'][1]), 'boxes': int(result['boxes'][1])},
                {'id': int(result['ids'][2]), 'boxes': int(result['boxes'][2])}

            ]
        }

    @app.route('/api/image')
    def get_result_img():
        return str("data:image/jpeg;base64," + result_img["img"])
    
    @app.route('/upload_audio', methods=['POST'])
    def upload_audio():
        print('Archivos recibidos: ', request.files)
        audio_file = request.files['audio']
        if not audio_file:
            return 'No se recibio archivo', 400
        save_path = os.path.join(UPLOAD_FOLDER, audio_file.filename)
        audio_file.save(save_path)
        
        
        audio_bytes = open(save_path, 'rb').read()
        base64_audio = base64.b64encode(audio_bytes).decode('utf-8')

        gateway_url = 'http://localhost:8042/getaudio'
        payload = {"audio": base64_audio}

        try:
            response = requests.post(gateway_url, json=payload)
        except Exception as e:
            print(f'Error: {e}')
            return 'Error al enviar al enviar al Gateway', 500
        
        return 'Audio recibido', 200
        
            

    @app.route('/')
    def show_result():
        return render_template('index_audio.html')

    app.run(debug=True, host='0.0.0.0', port=8002)
