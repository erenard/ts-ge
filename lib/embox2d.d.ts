declare module Box2D {

	export class b2BodyType {}
	export class b2_dynamicBody extends b2BodyType {}
	export class b2_kinematicBody extends b2BodyType {}
	export class b2_staticBody extends b2BodyType {}

	export class b2ShapeType {}
	export class e_circle extends b2ShapeType {}
	export class e_edge extends b2ShapeType {}
	export class e_polygon extends b2ShapeType {}
	export class e_chain extends b2ShapeType {}
	export class e_typeCount extends b2ShapeType {}

	export class b2JointType {}
	export class e_unknownJoint extends b2JointType {}
	export class e_revoluteJoint extends b2JointType {}
	export class e_prismaticJoint extends b2JointType {}
	export class e_distanceJoint extends b2JointType {}
	export class e_pulleyJoint extends b2JointType {}
	export class e_mouseJoint extends b2JointType {}
	export class e_gearJoint extends b2JointType {}
	export class e_wheelJoint extends b2JointType {}
	export class e_weldJoint extends b2JointType {}
	export class e_frictionJoint extends b2JointType {}
	export class e_ropeJoint extends b2JointType {}
	export class e_motorJoint extends b2JointType {}

	export class b2LimitState {}
	export class e_inactiveLimit extends b2LimitState {}
	export class e_atLowerLimit extends b2LimitState {}
	export class e_atUpperLimit extends b2LimitState {}
	export class e_equalLimits extends b2LimitState {}

	export class b2ManifoldType {}
	export class e_circles extends b2ManifoldType {}
	export class e_faceA extends b2ManifoldType {}
	export class e_faceB extends b2ManifoldType {}

	export interface b2Contact {
		GetManifold() : b2Manifold;
		IsTouching() : boolean;
		SetEnabled(flag : boolean) : void;
		IsEnabled() : boolean;
		GetNext() : b2Contact;
		GetFixtureA() : b2Fixture;
		GetChildIndexA() : number;
		GetFixtureB() : b2Fixture;
		GetChildIndexB() : number;
		SetFriction(friction : number) : void;
		GetFriction() : number;
		ResetFriction() : void;
		SetRestitution(restitution : number) : void;
		GetRestitution() : number;
		ResetRestitution() : void;
		SetTangentSpeed(speed : number) : void;
		GetTangentSpeed() : number;
	}

	export interface b2ContactListener {
		b2ContactListener : any;
	}

	export interface JSContactListener {
		JSContactListener() : void;
		BeginContact(contact : b2Contact) : void;
		EndContact(contact : b2Contact) : void;
		PreSolve(contact : b2Contact, oldManifold : b2Manifold) : void;
		PostSolve(contact : b2Contact, impulse : b2ContactImpulse) : void;
	}

	export class b2World {
		constructor(gravity : b2Vec2);
		SetDestructionListener(listener : b2DestructionListener) : void;
		SetContactFilter(filter : JSContactFilter) : void;
		SetContactListener(listener : JSContactListener) : void;
		SetDebugDraw(debugDraw : b2Draw) : void;
		CreateBody(def : b2BodyDef) : b2Body;
		DestroyBody(body : b2Body) : void;
		CreateJoint(def : b2JointDef) : b2Joint;
		DestroyJoint(joint : b2Joint) : void;
		Step(timeStep : number, velocityIterations : number, positionIterations : number) : void;
		ClearForces() : void;
		DrawDebugData() : void;
		QueryAABB(callback : b2QueryCallback, aabb : b2AABB) : void;
		RayCast(callback : b2RayCastCallback, point1 : b2Vec2, point2 : b2Vec2) : void;
		GetBodyList() : b2Body;
		GetJointList() : b2Joint;
		GetContactList() : b2Contact;
		SetAllowSleeping(flag : boolean) : void;
		GetAllowSleeping() : boolean;
		SetWarmStarting(flag : boolean) : void;
		GetWarmStarting() : boolean;
		SetContinuousPhysics(flag : boolean) : void;
		GetContinuousPhysics() : boolean;
		SetSubStepping(flag : boolean) : void;
		GetSubStepping() : boolean;
		GetProxyCount() : number;
		GetBodyCount() : number;
		GetJointCount() : number;
		GetContactCount() : number;
		GetTreeHeight() : number;
		GetTreeBalance() : number;
		GetTreeQuality() : number;
		SetGravity(gravity : b2Vec2) : void;
		GetGravity() : b2Vec2;
		IsLocked() : boolean;
		SetAutoClearForces(flag : boolean) : void;
		GetAutoClearForces() : boolean;
		GetProfile() : b2Profile;
		Dump() : void;
	}

	export class b2Shape {
		private m_type : b2ShapeType;
		public get_m_type() : b2ShapeType;
		public set_m_type(m_type : b2ShapeType): void;
		private m_radius : number;
		public get_m_radius() : number;
		public set_m_radius(m_radius : number): void;
		GetType() : b2ShapeType;
		GetChildCount() : number;
		TestPoint(xf : b2Transform, p : b2Vec2) : boolean;
		RayCast(output : b2RayCastOutput, input : b2RayCastInput, transform : b2Transform, childIndex : number) : boolean;
		ComputeAABB(aabb : b2AABB, xf : b2Transform, childIndex : number) : void;
		ComputeMass(massData : b2MassData, density : number) : void;
	}

	export class b2FixtureDef {
		private shape : b2Shape;
		public get_shape() : b2Shape;
		public set_shape(shape : b2Shape): void;
		private userData : any;
		public get_userData() : any;
		public set_userData(userData : any): void;
		private friction : number;
		public get_friction() : number;
		public set_friction(friction : number): void;
		private restitution : number;
		public get_restitution() : number;
		public set_restitution(restitution : number): void;
		private density : number;
		public get_density() : number;
		public set_density(density : number): void;
		private isSensor : boolean;
		public get_isSensor() : boolean;
		public set_isSensor(isSensor : boolean): void;
		private filter : b2Filter;
		public get_filter() : b2Filter;
		public set_filter(filter : b2Filter): void;
		constructor();
	}

	export interface b2Fixture {
		GetType() : b2ShapeType;
		GetShape() : b2Shape;
		SetSensor(sensor : boolean) : void;
		IsSensor() : boolean;
		SetFilterData(filter : b2Filter) : void;
		GetFilterData() : b2Filter;
		Refilter() : void;
		GetBody() : b2Body;
		GetNext() : b2Fixture;
		GetUserData() : any;
		SetUserData(data : any) : void;
		TestPoint(p : b2Vec2) : boolean;
		RayCast(output : b2RayCastOutput, input : b2RayCastInput, childIndex : number) : boolean;
		GetMassData(massData : b2MassData) : void;
		SetDensity(density : number) : void;
		GetDensity() : number;
		GetFriction() : number;
		SetFriction(friction : number) : void;
		GetRestitution() : number;
		SetRestitution(restitution : number) : void;
		GetAABB(childIndex : number) : b2AABB;
		Dump(bodyIndex : number) : void;
	}

	export class b2Transform {
		p : b2Vec2;
		q : b2Rot;
		constructor();
		constructor(position : b2Vec2, rotation : b2Rot);
		SetIdentity() : void;
		Set(position : b2Vec2, angle : number) : void;
	}

	export interface b2RayCastCallback {
		b2RayCastCallback : any;
	}

	export class JSRayCastCallback {
		constructor();
		ReportFixture(fixture : b2Fixture, point : b2Vec2, normal : b2Vec2, fraction : number) : number;
	}

	interface b2QueryCallback {
		b2QueryCallback : any;
	}

	export class JSQueryCallback {
		constructor();
		ReportFixture(fixture : b2Fixture) : boolean;
	}

	export class b2MassData {
		mass : number;
		center : b2Vec2;
		I : number;
		constructor();
	}

	export class b2Vec2 {
		x : number;
		y : number;
		constructor();
		constructor(x : number, y : number);
		SetZero() : void;
		Set(x : number, y : number) : void;
		op_add(v : b2Vec2) : void;
		op_sub(v : b2Vec2) : void;
		op_mul(s : number) : void;
		Length() : number;
		LengthSquared() : number;
		Normalize() : number;
		IsValid() : boolean;
		Skew() : b2Vec2;
	}

	export class b2Vec3 {
		x : number;
		y : number;
		z : number;
		constructor();
		constructor(x : number, y : number, z : number);
		SetZero() : void;
		Set(x : number, y : number, z : number) : void;
		op_add(v : b2Vec3) : void;
		op_sub(v : b2Vec3) : void;
		op_mul(s : number) : void;
	}

	export interface b2Body {
		CreateFixture(def : b2FixtureDef) : b2Fixture;
		CreateFixture(shape : b2Shape, density : number) : b2Fixture;
		DestroyFixture(fixture : b2Fixture) : void;
		SetTransform(position : b2Vec2, angle : number) : void;
		GetTransform() : b2Transform;
		GetPosition() : b2Vec2;
		GetAngle() : number;
		GetWorldCenter() : b2Vec2;
		GetLocalCenter() : b2Vec2;
		SetLinearVelocity(v : b2Vec2) : void;
		GetLinearVelocity() : b2Vec2;
		SetAngularVelocity(omega : number) : void;
		GetAngularVelocity() : number;
		ApplyForce(force : b2Vec2, point : b2Vec2, awake : boolean) : void;
		ApplyForceToCenter(force : b2Vec2, awake : boolean) : void;
		ApplyTorque(torque : number, awake : boolean) : void;
		ApplyLinearImpulse(impulse : b2Vec2, point : b2Vec2, awake : boolean) : void;
		ApplyAngularImpulse(impulse : number, awake : boolean) : void;
		GetMass() : number;
		GetInertia() : number;
		GetMassData(data : b2MassData) : void;
		SetMassData(data : b2MassData) : void;
		ResetMassData() : void;
		GetWorldPoint(localPoint : b2Vec2) : b2Vec2;
		GetWorldVector(localVector : b2Vec2) : b2Vec2;
		GetLocalPoint(worldPoint : b2Vec2) : b2Vec2;
		GetLocalVector(worldVector : b2Vec2) : b2Vec2;
		GetLinearVelocityFromWorldPoint(worldPoint : b2Vec2) : b2Vec2;
		GetLinearVelocityFromLocalPoint(localPoint : b2Vec2) : b2Vec2;
		GetLinearDamping() : number;
		SetLinearDamping(linearDamping : number) : void;
		GetAngularDamping() : number;
		SetAngularDamping(angularDamping : number) : void;
		GetGravityScale() : number;
		SetGravityScale(scale : number) : void;
		SetType(type : b2BodyType) : void;
		GetType() : b2BodyType;
		SetBullet(flag : boolean) : void;
		IsBullet() : boolean;
		SetSleepingAllowed(flag : boolean) : void;
		IsSleepingAllowed() : boolean;
		SetAwake(flag : boolean) : void;
		IsAwake() : boolean;
		SetActive(flag : boolean) : void;
		IsActive() : boolean;
		SetFixedRotation(flag : boolean) : void;
		IsFixedRotation() : boolean;
		GetFixtureList() : b2Fixture;
		GetJointList() : b2JointEdge;
		GetContactList() : b2ContactEdge;
		GetNext() : b2Body;
		GetUserData() : any;
		SetUserData(data : any) : void;
		GetWorld() : b2World;
		Dump() : void;
	}

	export class b2BodyDef {
		private type : b2BodyType;
		public get_type(): b2BodyType;
		public set_type(type : b2BodyType): void;
		private position : b2Vec2;
		public get_position(): b2Vec2;
		public set_position(position : b2Vec2): void;
		private angle : number;
		public get_angle(): number;
		public set_angle(angle : number): void;
		private linearVelocity : b2Vec2;
		public get_linearVelocity(): b2Vec2;
		public set_linearVelocity(linearVelocity : b2Vec2): void;
		private angularVelocity : number;
		public get_angularVelocity(): number;
		public set_angularVelocity(angularVelocity : number): void;
		private linearDamping : number;
		public get_linearDamping(): number;
		public set_linearDamping(linearDamping : number): void;
		private angularDamping : number;
		public get_angularDamping(): number;
		public set_angularDamping(angularDamping : number): void;
		private allowSleep : boolean;
		public get_allowSleep(): boolean;
		public set_allowSleep(allowSleep : boolean): void;
		private awake : boolean;
		public get_awake(): boolean;
		public set_awake(awake : boolean): void;
		private fixedRotation : boolean;
		public get_fixedRotation(): boolean;
		public set_fixedRotation(fixedRotation : boolean): void;
		private bullet : boolean;
		public get_bullet(): boolean;
		public set_bullet(bullet : boolean): void;
		private active : boolean;
		public get_active(): boolean;
		public set_active(active : boolean): void;
		private userData : any;
		public get_userData(): any;
		public set_userData(data : any): void;
		private gravityScale : number;
		public get_gravityScale(): number;
		public set_gravityScale(gravityScale : number): void;
		constructor();
	}

	export class b2Filter {
		categoryBits : number;
		maskBits : number;
		groupIndex : number;
		constructor();
	}

	interface b2AABB {
		lowerBound : b2Vec2;
		upperBound : b2Vec2;
		b2AABB() : void;
		IsValid() : boolean;
		GetCenter() : b2Vec2;
		GetExtents() : b2Vec2;
		GetPerimeter() : number;
		Combine(aabb : b2AABB) : void;
		Combine(aabb1 : b2AABB, aabb2 : b2AABB) : void;
		Contains(aabb : b2AABB) : boolean;
		RayCast(output : b2RayCastOutput, input : b2RayCastInput) : boolean;
	}

	interface b2CircleShape {
		m_p : b2Vec2;
		b2CircleShape() : void;
	}

	interface b2EdgeShape {
		m_vertex1 : b2Vec2;
		m_vertex2 : b2Vec2;
		m_vertex0 : b2Vec2;
		m_vertex3 : b2Vec2;
		m_hasVertex0 : boolean;
		m_hasVertex3 : boolean;
		b2EdgeShape() : void;
		Set(v1 : b2Vec2, v2 : b2Vec2) : void;
	}

	interface b2JointDef {
		type : b2JointType;
		userData : any;
		bodyA : b2Body;
		bodyB : b2Body;
		collideConnected : boolean;
		b2JointDef() : void;
	}

	interface b2Joint {
		GetType() : b2JointType;
		GetBodyA() : b2Body;
		GetBodyB() : b2Body;
		GetAnchorA() : b2Vec2;
		GetAnchorB() : b2Vec2;
		GetReactionForce(inv_dt : number) : b2Vec2;
		GetReactionTorque(inv_dt : number) : number;
		GetNext() : b2Joint;
		GetUserData() : any;
		SetUserData(data : any) : void;
		IsActive() : boolean;
		GetCollideConnected() : boolean;
		Dump() : void;
	}

	interface b2WeldJoint {
		GetLocalAnchorA() : b2Vec2;
		GetLocalAnchorB() : b2Vec2;
		SetFrequency(hz : number) : void;
		GetFrequency() : number;
		SetDampingRatio(ratio : number) : void;
		GetDampingRatio() : number;
		Dump() : void;
	}

	interface b2WeldJointDef {
		localAnchorA : b2Vec2;
		localAnchorB : b2Vec2;
		referenceAngle : number;
		frequencyHz : number;
		dampingRatio : number;
		b2WeldJointDef() : void;
		Initialize(bodyA : b2Body, bodyB : b2Body, anchor : b2Vec2) : void;
	}

	interface b2ChainShape {
		m_vertices : b2Vec2;
		m_count : number;
		m_prevVertex : b2Vec2;
		m_nextVertex : b2Vec2;
		m_hasPrevVertex : boolean;
		m_hasNextVertex : boolean;
		b2ChainShape() : void;
		Clear() : void;
		CreateLoop(vertices : b2Vec2, count : number) : void;
		CreateChain(vertices : b2Vec2, count : number) : void;
		SetPrevVertex(prevVertex : b2Vec2) : void;
		SetNextVertex(nextVertex : b2Vec2) : void;
		GetChildEdge(edge : b2EdgeShape, index : number) : void;
	}

	interface b2Color {
		r : number;
		g : number;
		b : number;
		b2Color() : void;
		b2Color(r : number, g : number, b : number) : void;
		Set(ri : number, gi : number, bi : number) : void;
	}

	interface b2ContactEdge {
		other : b2Body;
		contact : b2Contact;
		prev : b2ContactEdge;
		next : b2ContactEdge;
		b2ContactEdge() : void;
	}

	interface b2ContactFeature {
		indexA : any;
		indexB : any;
		typeA : any;
		typeB : any;
	}

	interface b2ContactFilter {
		b2ContactFilter : any;
	}

	interface JSContactFilter {
		JSContactFilter() : void;
		ShouldCollide(fixtureA : b2Fixture, fixtureB : b2Fixture) : boolean;
	}

	interface b2ContactID {
		cf : b2ContactFeature;
		key : number;
	}

	interface b2ContactImpulse {
		count : number;
	}

	interface b2DestructionListener {
		b2DestructionListener : any;
	}

	interface b2DestructionListenerWrapper {
		b2DestructionListenerWrapper : any;
	}

	interface JSDestructionListener {
		JSDestructionListener() : void;
		SayGoodbyeJoint(joint : b2Joint) : void;
		SayGoodbyeFixture(joint : b2Fixture) : void;
	}

	interface b2DistanceJoint {
		GetLocalAnchorA() : b2Vec2;
		GetLocalAnchorB() : b2Vec2;
		SetLength(length : number) : void;
		GetLength() : number;
		SetFrequency(hz : number) : void;
		GetFrequency() : number;
		SetDampingRatio(ratio : number) : void;
		GetDampingRatio() : number;
	}

	interface b2DistanceJointDef {
		localAnchorA : b2Vec2;
		localAnchorB : b2Vec2;
		length : number;
		frequencyHz : number;
		dampingRatio : number;
		b2DistanceJointDef() : void;
		Initialize(bodyA : b2Body, bodyB : b2Body, anchorA : b2Vec2, anchorB : b2Vec2) : void;
	}

	export interface b2Draw {
		SetFlags(flags : number) : void;
		GetFlags() : number;
		AppendFlags(flags : number) : void;
		ClearFlags(flags : number) : void;
		DrawPolygon(vertices : b2Vec2, vertexCount : number, color : b2Color) : void;
		DrawSolidPolygon(vertices : b2Vec2, vertexCount : number, color : b2Color) : void;
		DrawCircle(center : b2Vec2, radius : number, color : b2Color) : void;
		DrawSolidCircle(center : b2Vec2, radius : number, axis : b2Vec2, color : b2Color) : void;
		DrawSegment(p1 : b2Vec2, p2 : b2Vec2, color : b2Color) : void;
		DrawTransform(xf : b2Transform) : void;
	}

	interface b2FrictionJoint {
		GetLocalAnchorA() : b2Vec2;
		GetLocalAnchorB() : b2Vec2;
		SetMaxForce(force : number) : void;
		GetMaxForce() : number;
		SetMaxTorque(torque : number) : void;
		GetMaxTorque() : number;
	}

	interface b2FrictionJointDef {
		localAnchorA : b2Vec2;
		localAnchorB : b2Vec2;
		maxForce : number;
		maxTorque : number;
		b2FrictionJointDef() : void;
		Initialize(bodyA : b2Body, bodyB : b2Body, anchor : b2Vec2) : void;
	}

	interface b2GearJoint {
		GetJoint1() : b2Joint;
		GetJoint2() : b2Joint;
		SetRatio(ratio : number) : void;
		GetRatio() : number;
	}

	interface b2GearJointDef {
		joint1 : b2Joint;
		joint2 : b2Joint;
		ratio : number;
		b2GearJointDef() : void;
	}

	interface b2JointEdge {
		other : b2Body;
		joint : b2Joint;
		prev : b2JointEdge;
		next : b2JointEdge;
		b2JointEdge() : void;
	}

	interface b2Manifold {
		localNormal : b2Vec2;
		localPoint : b2Vec2;
		type : b2ManifoldType;
		pointCount : number;
		b2Manifold() : void;
	}

	interface b2ManifoldPoint {
		localPoint : b2Vec2;
		normalImpulse : number;
		tangentImpulse : number;
		id : b2ContactID;
		b2ManifoldPoint() : void;
	}

	interface b2Mat22 {
		ex : b2Vec2;
		ey : b2Vec2;
		b2Mat22() : void;
		b2Mat22(c1 : b2Vec2, c2 : b2Vec2) : void;
		b2Mat22(a11 : number, a12 : number, a21 : number, a22 : number) : void;
		Set(c1 : b2Vec2, c2 : b2Vec2) : void;
		SetIdentity() : void;
		SetZero() : void;
		GetInverse() : b2Mat22;
		Solve(b : b2Vec2) : b2Vec2;
	}

	interface b2Mat33 {
		ex : b2Vec3;
		ey : b2Vec3;
		ez : b2Vec3;
		b2Mat33() : void;
		b2Mat33(c1 : b2Vec3, c2 : b2Vec3, c3 : b2Vec3) : void;
		SetZero() : void;
		Solve33(b : b2Vec3) : b2Vec3;
		Solve22(b : b2Vec2) : b2Vec2;
		GetInverse22(M : b2Mat33) : void;
		GetSymInverse33(M : b2Mat33) : void;
	}

	interface b2MouseJoint {
		SetTarget(target : b2Vec2) : void;
		GetTarget() : b2Vec2;
		SetMaxForce(force : number) : void;
		GetMaxForce() : number;
		SetFrequency(hz : number) : void;
		GetFrequency() : number;
		SetDampingRatio(ratio : number) : void;
		GetDampingRatio() : number;
	}

	interface b2MouseJointDef {
		target : b2Vec2;
		maxForce : number;
		frequencyHz : number;
		dampingRatio : number;
		b2MouseJointDef() : void;
	}

	export class b2PolygonShape extends b2Shape {
		private m_centroid : b2Vec2;
		public get_m_centroid(): b2Vec2;
		public set_m_centroid(m_centroid: b2Vec2): void;
		private m_count : number;
		public get_m_count() : number;
		public set_m_count(m_count : number) : void;
		constructor();
		Set(vertices : b2Vec2, vertexCount : number) : void;
		SetAsBox(hx : number, hy : number) : void;
		SetAsBox(hx : number, hy : number, center : b2Vec2, angle : number) : void;
		GetVertexCount() : number;
		GetVertex(index : number) : b2Vec2;
	}

	interface b2PrismaticJoint {
		GetLocalAnchorA() : b2Vec2;
		GetLocalAnchorB() : b2Vec2;
		GetLocalAxisA() : b2Vec2;
		GetReferenceAngle() : number;
		GetJointTranslation() : number;
		GetJointSpeed() : number;
		IsLimitEnabled() : boolean;
		EnableLimit(flag : boolean) : void;
		GetLowerLimit() : number;
		GetUpperLimit() : number;
		SetLimits(lower : number, upper : number) : void;
		IsMotorEnabled() : boolean;
		EnableMotor(flag : boolean) : void;
		SetMotorSpeed(speed : number) : void;
		GetMotorSpeed() : number;
		SetMaxMotorForce(force : number) : void;
		GetMaxMotorForce() : number;
		GetMotorForce(inv_dt : number) : number;
	}

	interface b2PrismaticJointDef {
		localAnchorA : b2Vec2;
		localAnchorB : b2Vec2;
		localAxisA : b2Vec2;
		referenceAngle : number;
		enableLimit : boolean;
		lowerTranslation : number;
		upperTranslation : number;
		enableMotor : boolean;
		maxMotorForce : number;
		motorSpeed : number;
		b2PrismaticJointDef() : void;
		Initialize(bodyA : b2Body, bodyB : b2Body, anchor : b2Vec2, axis : b2Vec2) : void;
	}

	interface b2Profile {
		step : number;
		collide : number;
		solve : number;
		solveInit : number;
		solveVelocity : number;
		solvePosition : number;
		broadphase : number;
		solveTOI : number;
	}

	interface b2PulleyJoint {
		GetGroundAnchorA() : b2Vec2;
		GetGroundAnchorB() : b2Vec2;
		GetLengthA() : number;
		GetLengthB() : number;
		GetRatio() : number;
		GetCurrentLengthA() : number;
		GetCurrentLengthB() : number;
	}

	interface b2PulleyJointDef {
		groundAnchorA : b2Vec2;
		groundAnchorB : b2Vec2;
		localAnchorA : b2Vec2;
		localAnchorB : b2Vec2;
		lengthA : number;
		lengthB : number;
		ratio : number;
		b2PulleyJointDef() : void;
		Initialize(bodyA : b2Body, bodyB : b2Body, groundAnchorA : b2Vec2, groundAnchorB : b2Vec2, anchorA : b2Vec2, anchorB : b2Vec2, ratio : number) : void;
	}

	interface b2RayCastInput {
		p1 : b2Vec2;
		p2 : b2Vec2;
		maxFraction : number;
	}

	interface b2RayCastOutput {
		normal : b2Vec2;
		fraction : number;
	}

	interface b2RevoluteJoint {
		GetLocalAnchorA() : b2Vec2;
		GetLocalAnchorB() : b2Vec2;
		GetReferenceAngle() : number;
		GetJointAngle() : number;
		GetJointSpeed() : number;
		IsLimitEnabled() : boolean;
		EnableLimit(flag : boolean) : void;
		GetLowerLimit() : number;
		GetUpperLimit() : number;
		SetLimits(lower : number, upper : number) : void;
		IsMotorEnabled() : boolean;
		EnableMotor(flag : boolean) : void;
		SetMotorSpeed(speed : number) : void;
		GetMotorSpeed() : number;
		SetMaxMotorTorque(torque : number) : void;
		GetMaxMotorTorque() : number;
		GetMotorTorque(inv_dt : number) : number;
	}

	interface b2RevoluteJointDef {
		localAnchorA : b2Vec2;
		localAnchorB : b2Vec2;
		referenceAngle : number;
		enableLimit : boolean;
		lowerAngle : number;
		upperAngle : number;
		enableMotor : boolean;
		motorSpeed : number;
		maxMotorTorque : number;
		b2RevoluteJointDef() : void;
		Initialize(bodyA : b2Body, bodyB : b2Body, anchor : b2Vec2) : void;
	}

	interface b2RopeJoint {
		GetLocalAnchorA() : b2Vec2;
		GetLocalAnchorB() : b2Vec2;
		SetMaxLength(length : number) : void;
		GetMaxLength() : number;
		GetLimitState() : b2LimitState;
	}

	interface b2RopeJointDef {
		localAnchorA : b2Vec2;
		localAnchorB : b2Vec2;
		maxLength : number;
		b2RopeJointDef() : void;
	}

	interface b2Rot {
		s : number;
		c : number;
		b2Rot() : void;
		b2Rot(angle : number) : void;
		Set(angle : number) : void;
		SetIdentity() : void;
		GetAngle() : number;
		GetXAxis() : b2Vec2;
		GetYAxis() : b2Vec2;
	}

	interface b2WheelJoint {
		GetLocalAnchorA() : b2Vec2;
		GetLocalAnchorB() : b2Vec2;
		GetLocalAxisA() : b2Vec2;
		GetJointTranslation() : number;
		GetJointSpeed() : number;
		IsMotorEnabled() : boolean;
		EnableMotor(flag : boolean) : void;
		SetMotorSpeed(speed : number) : void;
		GetMotorSpeed() : number;
		SetMaxMotorTorque(torque : number) : void;
		GetMaxMotorTorque() : number;
		GetMotorTorque(inv_dt : number) : number;
		SetSpringFrequencyHz(hz : number) : void;
		GetSpringFrequencyHz() : number;
		SetSpringDampingRatio(ratio : number) : void;
		GetSpringDampingRatio() : number;
	}

	interface b2WheelJointDef {
		localAnchorA : b2Vec2;
		localAnchorB : b2Vec2;
		localAxisA : b2Vec2;
		enableMotor : boolean;
		maxMotorTorque : number;
		motorSpeed : number;
		frequencyHz : number;
		dampingRatio : number;
		b2WheelJointDef() : void;
		Initialize(bodyA : b2Body, bodyB : b2Body, anchor : b2Vec2, axis : b2Vec2) : void;
	}

	interface b2MotorJoint {
		SetLinearOffset(linearOffset : b2Vec2) : void;
		GetLinearOffset() : b2Vec2;
		SetAngularOffset(angularOffset : number) : void;
		GetAngularOffset() : number;
		SetMaxForce(force : number) : void;
		GetMaxForce() : number;
		SetMaxTorque(torque : number) : void;
		GetMaxTorque() : number;
		SetCorrectionFactor(factor : number) : void;
		GetCorrectionFactor() : number;
	}

	interface b2MotorJointDef {
		linearOffset : b2Vec2;
		angularOffset : number;
		maxForce : number;
		maxTorque : number;
		correctionFactor : number;
		b2MotorJointDef() : void;
		Initialize(bodyA : b2Body, bodyB : b2Body) : void;
	}

}
