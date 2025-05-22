import config from "./config.js";
export class Item{ 
    constructor(type){
        this.id = null;
        this.type = type;
        this.pos ={x:0, y:0};
        this.mesh = null;
        this.tweens = {};
        this.discarded = false;
        this.light = null;
        this.status = "Idle";
        this.screenLable = null;
    }
    setMesh(mesh){
        this.mesh = mesh;
    }
    setPos(pos){
        if(this.mesh != null){
            this.pos = pos;
            this.mesh.position.set((pos.x-0.5)*config.AREANA_DIM, 0.3, (pos.y-0.5)*config.AREANA_DIM); 
        }else{
            console.log("No mesh assigned with this instance")
        }
    }
}
