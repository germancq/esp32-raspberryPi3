from flask import Flask,request

app = Flask(__name__)

@app.route('/')
def index():
    return 'Hello world'

@app.route('/accelerometer', methods=['POST'])
def api_accelerometter():
    x = request.args.get('x')
    y = request.args.get('y')
    z = request.args.get('z')
    print(x)
    print(y)
    print(z)
    return 'Hello Accelerometer'

if __name__ == '__main__':
    app.run(debug=True, host = '0.0.0.0')
