# Sets up a web app for the data visualization.

import dash
from dash import html
from dash import dcc
import numpy as np

from dash import Input, Output, State, callback
from dash_extensions import WebSocket
# pip install dash dash-extensions quart --break-system

# update_graph = """
# function(msg) {
#     if(!msg) { return null; }  // no data, just return
#     const data = JSON.parse(msg.data);
#     let len = data.x.length;
#     let xs = data.x;
#     let ys = data.y;
#     let opacity = data.weight;

#     return [
#         {
#             x: [xs],
#             y: [ys],
#             'marker.opacity': [ opacity ],
#         },
#         [0],
#         len,
#     ];
# }
# """

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

dashApp = dash.Dash(__name__)


# Create small example app.
def start_web_server():
    figure = dict(
        data=[
            {
                'x': [0],
                'y': [0],
                'z': [],
                'xlim': [-72, 72],
                'ylim': [-72, 72],
                "mode": "markers",
                # "mode": "lines",
                "type": "scatter",
                "marker": {
                    "opacity": [1],
                }
            },
            # TODO: test whether this contour plot works properly with weights
            {
                'x': [0],
                'y': [0],
                'z': [0],
                'name': 'density',
                'ncontours': 20,
                'colorscale': 'Hot',
                'reversescale': True,
                'showscale': False,
                'type': 'histogram2dcontour',
                'marker': {
                    'opacity': [],
                },
                'histfunc': 'sum',
            },
        ],
        layout={
            'yaxis': {
                'scaleanchor': 'x',
                'scaleratio': 1,
                'range': [-72, 72],
            },
            "xaxis": {
                'range': [-72, 72],
            }
        })
    dashApp.layout = html.Div(
        [
            WebSocket(id="ws", url="ws://127.0.0.1:8766/particles"),
            dcc.Graph(id="graph", figure=dict(figure), className="viz-graph"),
        ],
        className="viz-app-container",
    )
    dashApp.clientside_callback(
        dash.ClientsideFunction(namespace='clientside',
                                function_name='on_websocket_message'),
        Output("graph", "extendData"), Input("ws", "message"))
    dashApp.clientside_callback(on_ws_state_change, [], Input("ws", "state"))

    dashApp.run(
        debug=True,
        dev_tools_hot_reload=False,
        use_reloader=False,
    )


def cleanup_app():
    pass
