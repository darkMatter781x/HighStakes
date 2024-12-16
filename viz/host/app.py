# Sets up a web app for the data visualization.

import os
import dash
from dash import dcc, Input, Output, html
from dash_extensions import WebSocket
import plotly
import plotly.graph_objects as go

# pip install dash dash-extensions quart --break-system

# Callback to handle websocket close event.
# Ideally, we should attempt to reconnect the websocket here,
# but I don't think this is possible with the WebSocket component.
on_ws_state_change = """
function(ws_state) {
    console.log("websocket state: ", ws_state);
    if(ws_state?.readyState == WebSocket.CLOSED) 
        alert("Websocket connection closed :(");
}
"""

# on_slider_change = """
# function(value) {
#     return [
#         {
#             "marker.angle": [[value]],
#         },
#         [2],
#         1
#     ]
# }
# """

dashApp = dash.Dash(
    __name__,
    assets_folder=os.path.dirname(os.path.realpath(__file__)) +
    "/../client/dist",
    # TODO: check if this syntax is correct
    assets_ignore=".*.ts|tsconfig.json|out/.*",
)

# Number of lasers to visualize.
# If changed, also change in viz/client/src/on_websocket_message.ts
num_of_lasers = 2


# Create small example app.
def start_web_server():
    particle_data = {
        'x': [0],
        'y': [0],
        'z': [],
        'xlim': [-72, 72],
        'ylim': [-72, 72],
        "mode": "markers",
        # "mode": "lines",
        "type": "scattergl",
        "marker": {
            "opacity": [1],
        }
    }
    single_laser_data = {
        'x': [0, 0],
        'y': [0, 0],
        'xlim': [-72, 72],
        'ylim': [-72, 72],
        "mode": "lines+markers",
        "type": "scatter",
    }

    laser_data = [single_laser_data.copy() for i in range(num_of_lasers)]
    for i in range(num_of_lasers):
        laser_data[i]['name'] = f"laser {i}"
        laser_data[i]['x'] = [0, 0]
        laser_data[i]['y'] = [0, 0]

    pose_data = {
        'x': [0],
        'y': [0],
        'xlim': [-72, 72],
        'ylim': [-72, 72],
        "mode": "markers",
        "type": "scatter",
        "marker": {
            "symbol": "triangle-up-open",
            "angle": [0],
            "size": [20],
        },
    }
    calculated_pose_data = pose_data.copy()
    calculated_pose_data['name'] = "calculated pose"

    actual_pose_data = pose_data.copy()
    actual_pose_data['name'] = "actual pose"

    # TODO: test whether this contour plot works properly with weights
    contour_data = dict(
        x=[0],
        y=[0],
        z=[0],
        name='density',
        ncontours=10,
        colorscale=[[0, 'rgba(0,0,255,1)'], [1, 'rgba(255,255,255,0)']],
        reversescale=True,
        showscale=True,
        type='histogram2dcontour',
        histfunc='sum',
        colorbar=go.histogram2dcontour.ColorBar(
            x=1,
            y=0,
            len=2. / 3.,
            yanchor='bottom',
        ),
        marker=dict(opacity=[]),
    )
    # contour_data["marker"]=dict(opacity=[]),

    layout = go.Layout(yaxis=go.layout.YAxis(
        scaleanchor='x',
        scaleratio=1,
        range=[-72, 72],
    ),
                       xaxis=go.layout.XAxis(range=[-72, 72], ),
                       legend=go.layout.Legend(
                           x=1,
                           y=1,
                       ))
    figure = dict(data=[
        particle_data,
        contour_data,
        calculated_pose_data,
        actual_pose_data,
        *laser_data,
    ],
                  layout=layout)
    dashApp.layout = html.Div(
        [
            # WebSocket(id="ws", url="ws://127.0.0.1:8766/data"),
            dcc.Graph(id="graphDiv", figure=dict(figure), className="viz-graph"),
            # dcc.Slider(id="angle-slider", min=0, max=360, step=1, value=0)
            html.Div(id="on_startup_placeholder", style={"display": "none"}),
        ],
        className="viz-app-container",
    )
    dashApp.clientside_callback(
        dash.ClientsideFunction(namespace='clientside',
                                function_name='on_startup'),
        Output("on_startup_placeholder", "children"),
        Input("on_startup_placeholder", "children"),
    )
    # dashApp.clientside_callback(
    #     dash.ClientsideFunction(namespace='clientside',
    #                             function_name='on_websocket_message'),
    #     Output("graph", "extendData"), Input("ws", "message"))
    # dashApp.clientside_callback(on_slider_change,
    #                             Output("graph", "extendData", allow_duplicate=True),
    #                             Input("angle-slider", "value"), prevent_initial_call=True)
    # dashApp.clientside_callback(on_ws_state_change, [], Input("ws", "state"))

    dashApp.run(
        debug=True,
        dev_tools_hot_reload=False,
        use_reloader=False,
    )


def cleanup_app():
    pass
