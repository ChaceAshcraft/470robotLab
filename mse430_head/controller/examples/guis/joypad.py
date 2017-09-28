#!/usr/bin/env python3

from kivy.app import App
from kivy.uix.floatlayout import FloatLayout
from kivy.graphics import Color, Ellipse
from kivy.uix.label import Label
from kivy.uix.widget import Widget

from collections import namedtuple
from math import sqrt, atan2

from remote import Remote

Point = namedtuple('Point', 'x y')


class Joypad(Widget):
    def __init__(self, app, *args, **kwargs):
        super(Joypad, self).__init__(**kwargs)
        self.app = app
        self.remote = Remote(app)
        self.anchor = None
        self.outer_r = 120.
        self.inner_r = 30.
        self.max_d = self.outer_r - self.inner_r
        self.max_speed = 500.

    def on_touch_down(self, touch):
        if not self.collide_point(*touch.pos):
            return False

        self.anchor = Point(touch.x, touch.y)
        self.draw_joystick(0, 0)
        self.remote.set_power((0, 0))

    def on_touch_move(self, touch):

        if not self.collide_point(*touch.pos):
            return False

        dx = touch.x - self.anchor.x
        dy = touch.y - self.anchor.y
        mag = sqrt(dx ** 2 + dy ** 2)
        if mag > self.max_d:
            dx /= mag / self.max_d
            dy /= mag / self.max_d

        self.remote.set_power(
            (int((dy + .3*dx) / self.max_d * self.max_speed),
             int((dy - .3*dx) / self.max_d * self.max_speed)))

        self.draw_joystick(dx, dy)
        return True

    def on_touch_up(self, touch):

        self.canvas.clear()
        self.remote.set_power((0, 0))

    def draw_joystick(self, dx, dy):
        self.canvas.clear()
        with self.canvas:
            Color(.5, .5, .5)
            d = 2 * self.outer_r
            Ellipse(pos=(self.anchor.x - d / 2, self.anchor.y - d / 2),
                    size=(d, d))

            Color(.6, .1, .1)
            d = 2 * self.inner_r
            Ellipse(
                pos=((self.anchor.x + dx) - d / 2,
                     (self.anchor.y + dy) - d / 2),
                size=(d, d))


class RobotRemote(App):
    def __init__(self, *args, **kwargs):
        super(RobotRemote, self).__init__(**kwargs)
        self.joypad = None
        self.a_cell = None
        self.b_cell = None

    def build(self):
        parent = FloatLayout(size_hint=(1, 1,))

        self.joypad = Joypad(self)
        parent.add_widget(self.joypad)

        self.a_cell = Label(
            text='Motor A',
            halign='center',
            size_hint=(.3, .1),
            pos_hint={'x': 0, 'y': .1})
        self.b_cell = Label(
            text='Motor B',
            halign='center',
            size_hint=(.3, .1),
            pos_hint={'x': .7, 'y': .1})
        parent.add_widget(self.a_cell)
        parent.add_widget(self.b_cell)

        return parent


if __name__ == '__main__':
    RobotRemote().run()
