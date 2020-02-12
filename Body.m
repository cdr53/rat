classdef Body < matlab.mixin.SetGet
    properties
        name
        id
        position
        rotation
        density
        mass
        com
        meshfile
        convexmeshfile
        scale
    end
    methods
        function obj = Body(Bname)
            obj.name = Bname;
        end
    end       
end