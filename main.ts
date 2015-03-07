import Engine = require("./Engine");


function createBox(point: Box2D.b2Vec2, force: Box2D.b2Vec2) {
    var bodyDef = new Box2D.b2BodyDef(),
        shape = new Box2D.b2PolygonShape(),
        fixtureDef = new Box2D.b2FixtureDef(),
        body;

    bodyDef.set_position(point);
    bodyDef.set_type(Box2D.b2_dynamicBody);
    bodyDef.set_linearDamping(0.1);
    bodyDef.set_angularDamping(0.1);

    shape.SetAsBox(1.0, 1.0);

    fixtureDef.set_density(0.1);
    fixtureDef.set_friction(0.3); //Resistance à la glissade
    fixtureDef.set_restitution(0.2);

    fixtureDef.set_shape(shape);

    body = world.CreateBody(bodyDef);
    body.CreateFixture(fixtureDef);
    body.ApplyForce(force, point);
}




var renderer = new Engine.Box2DRenderer(640, 480);
var world = renderer.createWorld(new Box2D.b2Vec2(0.0, -9.8));
Engine.Animator.setRenderer(renderer);
Engine.Animator.start();
createBox(new Box2D.b2Vec2(0, 0), new Box2D.b2Vec2(0, 0));
window.setTimeout(Engine.Animator.stop, 3000);