import inspect
import math

import pygame
import pygame.color
from pygame.locals import *

import pymunk
from pymunk.vec2d import Vec2d
import pymunk.pygame_util

pymunk.pygame_util.positive_y_is_up = False

pygame.init()
screen = pygame.display.set_mode((600, 600))
clock = pygame.time.Clock()
font = pygame.font.Font(None, 24)

help_txt = font.render(
    "Pymunk constraints demo. Use mouse to drag/drop. Hover to see descr.",
    1, pygame.color.THECOLORS["darkgray"])

space = pymunk.Space()
space.gravity = (0.0, 0.0)
draw_options = pymunk.pygame_util.DrawOptions(screen)


def add_ball(space, pos):
    body = pymunk.Body()
    body.position = Vec2d(pos)
    shape = pymunk.Circle(body, 20)
    shape.mass = 1
    shape.friction = 1
    space.add(body, shape)
    return body


def add_manipulator(space, pos):
    body = pymunk.Body()
    body.position = Vec2d(pos)
    a = body.position - Vec2d(60, 0)
    b = body.position + Vec2d(60, 0)
    shape = pymunk.Segment(body, a=a, b=b, radius=5)
    shape.mass = 10000
    space.add(body, shape)
    return body

# containers
w = screen.get_width()
h = screen.get_height()
dw = 0.1*w
dh = 0.1*h
sw = pymunk.Segment(space.static_body, (0, h), (w+dw, h+dh), 1)
sw.friction = 1
sw.elasticity = 1

sh = pymunk.Segment(space.static_body, (0, 0), (w+dw, 0), 1)
sh.friction = 1
sh.elasticity = 1

lh = pymunk.Segment(space.static_body, (0, 0), (0, h+dh), 1)
lh.friction = 1
lh.elasticity = 1

rh = pymunk.Segment(space.static_body, (w, 0), (w+dw, h+dh), 1)
rh.friction = 1
rh.elasticity = 1

space.add(sw, sh, lh, rh)

b1 = add_ball(space, (300, 300))
b2 = add_ball(space, (350, 300))
c = pymunk.PinJoint(b1, b2, (20, 0), (-20, 0))
space.add(c)

manipulator = add_manipulator(space, (150, 100))


mouse_joint = None
mouse_body = pymunk.Body(body_type=pymunk.Body.KINEMATIC)

while True:
    for event in pygame.event.get():
        if event.type == QUIT:
            exit()
        elif event.type == KEYDOWN and event.key == K_ESCAPE:
            exit()
        elif event.type == KEYDOWN and event.key == K_UP:
            manipulator.position += Vec2d(0, -10)
        elif event.type == KEYDOWN and event.key == K_DOWN:
            manipulator.position += Vec2d(0, 10)
        elif event.type == KEYDOWN and event.key == K_LEFT:
            manipulator.position += Vec2d(-10, 0)
        elif event.type == KEYDOWN and event.key == K_RIGHT:
            manipulator.position += Vec2d(10, 0)

        elif event.type == MOUSEBUTTONDOWN:
            if mouse_joint != None:
                space.remove(mouse_joint)
                mouse_joint = None

            p = Vec2d(event.pos)
            hit = space.point_query_nearest(p, 5, pymunk.ShapeFilter())
            if hit != None and hit.shape.body.body_type == pymunk.Body.DYNAMIC:
                shape = hit.shape
                # Use the closest point on the surface if the click is outside
                # of the shape.
                if hit.distance > 0:
                    nearest = hit.point
                else:
                    nearest = p
                mouse_joint = pymunk.PivotJoint(mouse_body, shape.body,
                                                (0, 0), shape.body.world_to_local(nearest))
                mouse_joint.max_force = 50000
                mouse_joint.error_bias = (1 - 0.15) ** 60
                space.add(mouse_joint)

        elif event.type == MOUSEBUTTONUP:
            if mouse_joint != None:
                space.remove(mouse_joint)
                mouse_joint = None

    for b in space.bodies:
        b.force = -b.velocity

    screen.fill(pygame.color.THECOLORS["white"])

    screen.blit(help_txt, (5, screen.get_height() - 20))

    mouse_pos = pygame.mouse.get_pos()

    mouse_body.position = mouse_pos

    space.step(1. / 60)

    space.debug_draw(draw_options)
    pygame.display.flip()

    clock.tick(60)
    pygame.display.set_caption("fps: " + str(clock.get_fps()))
