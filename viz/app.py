# Sets up a web app for the data visualization.

import dash
from dash import html
from dash import dcc
import numpy as np

from dash import Input, Output, State, callback
from dash_extensions import WebSocket
# pip install dash dash-extensions quart --break-system

update_graph = """
function(msg) {
    if(!msg) { return null; }  // no data, just return
    const data = JSON.parse(msg.data);
    let len = data.x.length;
    let xs = data.x;
    let ys = data.y;
    let opacity = data.weight;

    return [
        {
            x: [xs],
            y: [ys],
            'marker.opacity': [ opacity ],
        }, 
        [0], 
        len,
    ];
}
"""

dashApp = dash.Dash(__name__)


# Create small example app.
def start_web_server():
    figure = dict(
        data=[{
            'x': [0],
            'y': [0],
            'xlim': [-3, 3],
            'ylim': [-3, 3],
            "mode": "markers",
            # "mode": "lines",
            "type": "scatter",
            "marker": {
                "opacity": [1],
            }
        }],
        layout={
            'yaxis': {
                'scaleanchor': 'x',
                'scaleratio': 1,
                'range': [-3, 3],
            },
            "xaxis": {
                'range': [-3, 3],
            }
        })
    dashApp.layout = html.Div([
        WebSocket(id="ws", url="ws://127.0.0.1:8766/particles"),
        dcc.Graph(id="graph", figure=dict(figure), className="my-graph")
    ])
    dashApp.clientside_callback(update_graph, Output("graph", "extendData"),
                                Input("ws", "message"))

    dashApp.run(
        debug=True,
        dev_tools_hot_reload=False,
        use_reloader=False,
    )


def cleanup_app():
    pass
