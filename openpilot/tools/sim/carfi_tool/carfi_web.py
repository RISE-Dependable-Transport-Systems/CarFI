from multiprocessing import Queue
import flask
from flask import Flask, render_template, redirect, url_for, copy_current_request_context, request, Response
from flask_bootstrap import Bootstrap
from flask_wtf import FlaskForm
from wtforms import StringField, SubmitField, SelectField, IntegerField, SelectMultipleField, BooleanField, FloatField
from wtforms.validators import DataRequired, InputRequired
import util
import carfi
from flask_table import Table, Col
import threading
from turbo_flask import Turbo
import time
import numpy as np
import cv2
from PIL import Image
app = Flask(__name__)
turbo = Turbo(app)
app.config['SECRET_KEY'] = 'rlLn3PYeAVKmacjkFa9cXHa5HRxiLF7A'
app.config['WTF_CSRF_ENABLED'] = False
Bootstrap(app)

carfi_config = util.get_config(True)
logger = util.get_logger()
image_q = Queue()


class LogTable(Table):
    border = False
    classes = ['table', 'table-bordered']
    DateTime = Col('DateTime')
    Level = Col('Level')
    SimRound = Col('SimRound')
    SimTime = Col('SimTime')
    Event = Col('Event')


class HomeForm(FlaskForm):
    run = SubmitField('Setup a Campaign')
    res = SubmitField('Show Analysis Results')


class RunForm(FlaskForm):
    home = SubmitField('Home')
    analysis = SubmitField('Analyze Measures')


class ResultForm(FlaskForm):
    result = SelectField('Experiments on')
    show = SubmitField('Show Results')
    home = SubmitField('Home')
    download = SubmitField('Download Results')


class Config0Form(FlaskForm):
    pass


class Config1Form(FlaskForm):
    pass


@ app.route('/', methods=['GET', 'POST'])
def home():

    form = HomeForm()

    if form.validate_on_submit():
        if form.run.data:
            return redirect(url_for('config', id=0))
        elif form.res.data:
            return redirect(url_for('results', id='latest'))

    return render_template('home.html', form=form)


def config_0():
    for key, value in carfi_config.items():
        if isinstance(value, bool):
            setattr(Config0Form, key, BooleanField(key))
        elif isinstance(value, int):
            setattr(Config0Form, key, IntegerField(key, [InputRequired()]))
        elif isinstance(value, float):
            setattr(Config0Form, key, FloatField(key, [InputRequired()]))
        elif isinstance(value, dict):
            if 'value list' in value:
                setattr(Config0Form, key, SelectField(key))
    setattr(Config0Form, 'back', SubmitField('Home'))
    setattr(Config0Form, 'next', SubmitField('Next'))
    form = Config0Form()

    for key, value in carfi_config.items():
        if isinstance(value, bool) or isinstance(value, int) or isinstance(value, float):
            if request.method == 'GET':
                form[key].data = value
        elif isinstance(value, dict):
            if 'value list' in value:
                form[key].choices = value['value list']
                if request.method == 'GET':
                    form[key].data = value['value']

    if request.method == 'POST' and form.validate():

        for key in carfi_config:
            value = carfi_config[key]
            if isinstance(value, bool) or isinstance(value, int) or isinstance(value, float):
                carfi_config[key] = form[key].data
            elif isinstance(value, dict):
                if 'value list' in value:
                    carfi_config[key]['value'] = form[key].data
        selected_fault = carfi_config['fault type']['value']

        fault_params = util.extract_fault_params(selected_fault)

        if set(carfi_config['fault params'].keys()) != fault_params:

            carfi_config['fault params'] = dict()
            for fp in fault_params:
                carfi_config['fault params'][fp] = {'value': None, 'value range': None}
        selected_scenario = carfi_config['scenario name']['value']
        scenario_params = util.extract_scenario_params(selected_scenario)

        if set(carfi_config['scenario params'].keys()) != scenario_params:

            carfi_config['scenario params'] = dict()
            for sp in scenario_params:
                carfi_config['scenario params'][sp] = {'value': None, 'value range': None}
        if form.next.data:

            return redirect(url_for('config', id=1))

        elif form.back.data:

            return redirect('/')
    return render_template('config.html', form=form)


def config_1():
    for key, value in carfi_config.items():
        if isinstance(value, dict):
            if 'value range' in value:
                setattr(Config1Form, key, StringField(key+' (fixed or range)', [DataRequired()]))
            elif 'choice list' in value:

                setattr(Config1Form, key, SelectMultipleField(key, choices=value['choice list']))
            elif key == 'fault params':

                for fp in value:
                    setattr(Config1Form, fp, StringField('fault '+fp+' (fixed or range)', [DataRequired()]))
            elif key == 'scenario params':
                for sp in value:
                    setattr(Config1Form, sp, StringField('scenario '+sp+' (fixed or range)', [DataRequired()]))
    setattr(Config1Form, 'back', SubmitField('Back'))
    setattr(Config1Form, 'save', SubmitField('Save & Run'))
    setattr(Config1Form, 'next', SubmitField('Run'))
    form = Config1Form()
    if request.method == 'GET':
        for key, value in carfi_config.items():
            if isinstance(value, dict):
                if key == 'fault params' or key == 'scenario params':
                    for p in value:

                        form[p].data = value[p]['value'] if value[p]['value'] else value[p]['value range']
                elif 'value range' in value:
                    form[key].data = value['value'] if value['value'] else value['value range']
                elif 'choice list' in value:
                    form[key].data = value['chosen']

    if form.validate_on_submit():

        for key in carfi_config:
            value = carfi_config[key]
            if isinstance(value, dict):
                if 'value range' in value:

                    data = eval(form[key].data)

                    if isinstance(data, list):
                        carfi_config[key]['value range'] = util.ValueRange(data[0], data[1])
                        carfi_config[key]['value'] = None
                    else:
                        carfi_config[key]['value range'] = None
                        carfi_config[key]['value'] = data

                elif 'choice list' in value:
                    carfi_config[key]['chosen'] = form[key].data
                elif key == 'fault params' or key == 'scenario params':
                    for p in value:
                        data = eval(form[p].data)

                        if isinstance(data, list):
                            carfi_config[key][p]['value range'] = util.ValueRange(data[0], data[1])
                            carfi_config[key][p]['value'] = None
                        else:
                            carfi_config[key][p]['value range'] = None
                            carfi_config[key][p]['value'] = data

        if form.save.data:
            util.save_config()

            return redirect(url_for('run'))
        elif form.next.data:

            return redirect(url_for('run'))
        elif form.back.data:
            return redirect(url_for('config', id=0))
    return render_template('config.html', form=form)


@ app.route('/config/<id>', methods=['GET', 'POST'])
def config(id):
    if id == '0':
        return config_0()
    return config_1()


@app.context_processor
def inject_log():
    table = None
    table_data = None
    done = False
    total_rounds = carfi_config['# experiments']
    sim_round = None
    if request.path.startswith('/run'):

        with open(util.log_path(), 'r') as f:
            table_data = [eval(ev) for ev in f.readlines() if 'Event' in ev]

            if len(table_data) > 0:
                table_data.reverse()
                if len(table_data) > 4:
                    table_data = table_data[:4]
                table = LogTable(table_data)
                sim_round = table_data[0]['SimRound']
                done = sim_round == total_rounds and table_data[0]['Event'] in ['Scenario timeout',
                                                                                'Collision detected']
    return dict(done=done, table=table, sim_round=sim_round, total_rounds=total_rounds)


@ app.route('/run', methods=['GET', 'POST'])
def run():
    form = RunForm()

    @ copy_current_request_context
    def update_log():
        with app.app_context():
            while True:
                time.sleep(1)
                turbo.push(turbo.replace(render_template('log.html', form=form), 'log'))
    if request.method == 'GET':
        threading.Thread(target=carfi.run_sim, args=[image_q]).start()

        threading.Thread(target=update_log).start()

    if request.method == 'POST':
        if form.home.data:
            return redirect('/')
        elif form.analysis.data:
            return redirect(url_for('results', id='latest'))
    return render_template('run.html')


@ app.route('/results/<id>', methods=['GET', 'POST'])
def results(id):

    form = ResultForm()

    if request.method == 'GET':
        output_files = util.all_files('./out', 'csv', with_ext=False)
        output_dates = util.outputs_to_dates(output_files)
        form.result.choices = output_dates
        form.result.data = str(output_dates[-1]) if id == 'latest' else id
        d = next(i for i in output_dates if str(i) == form.result.data)

        carfi.analyze_measures(carfi_config['measures']['chosen'], util.date_to_output(d))

    image_files = util.all_files('./out', 'png')
    small_images = []
    large_images = []
    for image_file in image_files:
        im = Image.open('./out/'+image_file)
        if im.size[0] > 1000:
            large_images.append(image_file)
        else:
            small_images.append(image_file)
    if request.method == 'POST':

        if form.home.data:

            return redirect('/')
        elif form.show.data:
            return redirect(url_for('results', id=form.result.data))
        elif form.download.data:

            zip_archive = util.compress_files('./out/', image_files)
            response = flask.make_response(zip_archive.read())
            response.headers.set('Content-Type', 'zip')
            response.headers.set('Content-Disposition', 'attachment', filename='carfi_results.zip')
            return response
    return render_template('results.html', form=form, small_images=small_images, large_images=large_images)


@ app.route('/charts/<id>')
def charts(id):
    return flask.send_from_directory('./out', id, as_attachment=True)


def gen_frame():
    while True:
        time.sleep(0.045)
        frame = np.zeros([100, 100, 3], dtype=np.uint8)
        if not image_q.empty():
            frame = image_q.get()

        _, image = cv2.imencode('.jpg', frame)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + image.tobytes() + b'\r\n')


@app.route('/video_feed')
def video_feed():
    return Response(gen_frame(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    app.run(debug=True)
