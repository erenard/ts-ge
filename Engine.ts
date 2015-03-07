/// <reference path="lib/embox2d.d.ts"/>
/// <reference path="lib/embox2d-html5canvas-debugDraw.d.ts"/>

var requestAnimFrame: (callback: () => void) => void = (function(){ 
	return window.requestAnimationFrame || 
	(<any>window).webkitRequestAnimationFrame || 
	(<any>window).mozRequestAnimationFrame || 
	(<any>window).oRequestAnimationFrame || 
	window.msRequestAnimationFrame || 
	function(callback) {
		window.setTimeout(callback, 1000 / 60, new Date().getTime()); 
	}; 
})();

var cancelAnimFrame: (callback: () => void) => void = (function(){ 
	return window.cancelAnimationFrame || 
	(<any>window).webkitCancelAnimationFrame || 
	(<any>window).mozCancelAnimationFrame || 
	(<any>window).oCancelAnimationFrame || 
	(<any>window).msCancelAnimationFrame || 
	function(callback) {
		window.clearTimeout(callback);
	}; 
})();

export class Animator {
	private static renderer  : Renderer;
	private static rendering : boolean = false;

	public static setRenderer(renderer: Renderer): void {
		Animator.renderer = renderer;
	}

	public static start(): void {
		if(Animator.renderer) {
			console.debug("Animator.start");
			Animator.rendering = true;
			Animator.render();
		}
	}

	public static stop(): void {
		console.debug("Animator.stop");
		Animator.rendering = false;
	}

	private static render(): void {
		Animator.renderer.render.apply(Animator.renderer);
		if (Animator.rendering) {
			requestAnimFrame( Animator.render );
		} else {
			cancelAnimFrame( Animator.render );
		}
	}
}


export class Renderer {
	protected canvas	:	HTMLCanvasElement;
	protected context	:	CanvasRenderingContext2D;

	constructor(width: number, height: number) {
		this.canvas = document.createElement('canvas');
		this.canvas.width = width;
		this.canvas.height = height;
		this.canvas.style.border = 'solid 1px black';
		document.body.appendChild(this.canvas);
		this.context = this.canvas.getContext('2d');
	}

	public render(): void {
		var context = this.context;
		context.clearRect(0, 0, 640, 480);
		context.beginPath();
		context.moveTo(160, 120);
		context.lineTo(480, 120);
		context.lineTo(480, 360);
		context.lineTo(160, 360);
		context.closePath();
		context.stroke();
		context.fill();
	}
}

export class Box2DRenderer extends Renderer {
	public world:		Box2D.b2World;
	public debugDraw:	Box2D.b2Draw;
	private PTM: 		number = 8; //Pixel to meter
	private canvasOffset = {
        x: this.canvas.width / 2,
        y: this.canvas.height / 2
    };


	public createWorld(gravity: Box2D.b2Vec2): Box2D.b2World {
		this.world = new Box2D.b2World(gravity);
		this.debugDraw = getCanvasDebugDraw();
		this.world.SetDebugDraw(this.debugDraw);
		this.debugDraw.SetFlags(0x0001);
		return this.world;
	}

	public destroyWorld(): void {
		delete this.world;
	}
	
	public render(): void {
		var context = this.context;
		context.clearRect(0, 0, this.canvas.width, this.canvas.height);
        context.save();
        context.translate(this.canvasOffset.x, this.canvasOffset.y);
        context.scale(1, -1);
        context.scale(this.PTM, this.PTM);
        context.lineWidth /= this.PTM;
        drawAxes(context);
		if (this.world) {
			this.world.Step(1 / 60, 10, 8);
			this.world.ClearForces();
			this.world.DrawDebugData();
		}
        context.restore();
	}
}
