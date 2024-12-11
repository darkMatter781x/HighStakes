# Sets up a web app for the data visualization.

import os
import dash
from dash import dcc, Input, Output, html
from dash_extensions import WebSocket
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

dashApp = dash.Dash(
    __name__,
    assets_folder=os.path.dirname(os.path.realpath(__file__)) + "/../client/dist",
    # TODO: check if this syntax is correct
    assets_ignore=".*.ts|tsconfig.json|out/.*",
)


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
