from nicegui import ui, app


def create() -> None:
    app.add_static_files('/js', './src/web_dashboard/web_dashboard/static/js')

    @ui.page('/tree')
    def tree_page():
        with ui.header(elevated=False).style('background-color: #3874c8; height: 20px').classes('items-center justify-between'):
            ui.label('HEADER').style('flex-grow: 1;height: 20px')
            ui.button('Move!', on_click=lambda e: e).style('flex-shrink: 0;')
        with open('./src/web_dashboard/web_dashboard/static/index.html') as f:
            code = f.read()
            ui.add_body_html(code)
