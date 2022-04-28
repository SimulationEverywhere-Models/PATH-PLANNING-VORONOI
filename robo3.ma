[top]
components : robo

[robo]
type : cell
% plane 0 is the initial bitmap of the area
% plane 1 is the initial mapping of bitmaps into edge codes
% plane 2 is the propagation of edge codes across the plane
% plane 3 is the final Voronoi diagram
dim : (10, 10, 4)
delay : transport
defaultDelayTime  : 10
border : nowrapped 
neighbors :              robo(-1,0,0)
neighbors : robo(0,-1,0) robo(0,0,0) robo(0,1,0)
neighbors :              robo(1,0,0)
neighbors :     robo(0,-1,-1)    robo(-1,0,-1)
neighbors :           robo(0,0,-1)
neighbors :         robo(1,0,-1)     robo(0,1,-1)

initialvalue : 0
initialcellsvalue : robot.val

localtransition : nothing-rule
zone : bound-rule { (0,0,1)..(9,9,1) }
zone : plane2-rule { (0,0,2)..(9,9,2) }
zone : plane3-rule { (0,0,3)..(9,9,3) }

[plane3-rule]
% these rules build the final Voronoi diagram.  The rule is: if more than one flag is set in the
% cell neighborhood, then compare the edge code values -- if they are not the same, then this is the
% intersection of two expanding wavefronts, and the cell is part of the Voronoi diagram.
rule : {(time/10)} 10 { (0,0,0)=0 and fractional((-1,0,-1))=0.1 and fractional((0,1,-1))=0.1
                        and (-1,0,-1)!=(0,1,-1) }
rule : {(time/10)} 10 { (0,0,0)=0 and fractional((1,0,-1))=0.1 and fractional((0,1,-1))=0.1
                        and (1,0,-1)!=(0,1,-1) }
rule : {(time/10)} 10 { (0,0,0)=0 and fractional((-1,0,-1))=0.1 and fractional((0,-1,-1))=0.1
                        and (-1,0,-1)!=(0,-1,-1) }
rule : {(time/10)} 10 { (0,0,0)=0 and fractional((-1,0,-1))=0.1 and fractional((1,0,-1))=0.1
                        and (-1,0,-1)!=(1,0,-1) }
rule : {(time/10)} 10 { (0,0,0)=0 and fractional((1,0,-1))=0.1 and fractional((0,-1,-1))=0.1
                        and (1,0,-1)!=(0,-1,-1) }
rule : {(time/10)} 10 { (0,0,0)=0 and fractional((0,-1,-1))=0.1 and fractional((0,1,-1))=0.1
                        and (0,-1,-1)!=(0,1,-1) }

% do we need to consider any other possibilities?

rule : {(0,0,0)} 10 { t }

[plane2-rule]
% copy the value from the boundary detection phase if the edge codes are
% between 5 to 12... and flag it
rule : {(0,0,-1)+0.1} 10 { (0,0,-1) >4 and (0,0,-1)<13 }

% next, we go across the grid looking for neighbors with flags... we
% detect flags by the presence of that fractional part.
% we do not carry the flag over too!  (but I think we should...?)

rule : {(-1,0,0)} 10 { fractional((-1,0,0))=0.1 and isint((0,-1,0)) and
                            isint((0,1,0)) and isint((1,0,0)) }
rule : {(1,0,0)} 10 { fractional((1,0,0))=0.1 and isint((0,-1,0)) and
                            isint((0,1,0)) and isint((-1,0,0)) }
rule : {(0,-1,0)} 10 { fractional((0,-1,0))=0.1 and isint((-1,0,0)) and
                            isint((0,1,0)) and isint((1,0,0)) }
rule : {(0,1,0)} 10 { fractional((0,1,0))=0.1 and isint((0,-1,0)) and
                            isint((-1,0,0)) and isint((1,0,0)) }

rule : {(0,0,0)} 10 { t }

[nothing-rule]
rule : { (0,0,0) } 10 { t }

[bound-rule]
% this represents the edge-code stage in the object boundary detection
% phase... as in the IEEE paper, the edges are coded 1-12

rule : 1 10 { (0,0,-1)=1 and (0,-1,-1)=1 and (-1,0,-1)=1 and (0,1,-1)=1 and (1,0,-1)=1 } 
rule : 2 10 { (0,0,-1)=0 and (0,-1,-1)=0 and (-1,0,-1)=0 and (0,1,-1)=0 and (1,0,-1)=0 } 
rule : 3 10 { (0,0,-1)=0 and (0,-1,-1)=1 and (-1,0,-1)=1 and (0,1,-1)=1 and (1,0,-1)=1 } 
rule : 4 10 { (0,0,-1)=1 and (0,-1,-1)=0 and (-1,0,-1)=0 and (0,1,-1)=0 and (1,0,-1)=0 } 
rule : 5 10 { (0,0,-1)=1 and (0,-1,-1)=1 and (-1,0,-1)=1 and (0,1,-1)=1 and (1,0,-1)=0 } 
rule : 6 10 { (0,0,-1)=1 and (0,-1,-1)=1 and (-1,0,-1)=1 and (0,1,-1)=0 and (1,0,-1)=1 } 
rule : 7 10 { (0,0,-1)=1 and (0,-1,-1)=1 and (-1,0,-1)=1 and (0,1,-1)=0 and (1,0,-1)=0 } 
rule : 8 10 { (0,0,-1)=1 and (0,-1,-1)=0 and (-1,0,-1)=1 and (0,1,-1)=1 and (1,0,-1)=0 } 
rule : 9 10 { (0,0,-1)=1 and (0,-1,-1)=1 and (-1,0,-1)=0 and (0,1,-1)=1 and (1,0,-1)=1 } 
rule : 10 10 { (0,0,-1)=1 and (0,-1,-1)=0 and (-1,0,-1)=1 and (0,1,-1)=1 and (1,0,-1)=1 }
rule : 11 10 { (0,0,-1)=1 and (0,-1,-1)=0 and (-1,0,-1)=0 and (0,1,-1)=1 and (1,0,-1)=1 } 
rule : 12 10 { (0,0,-1)=1 and (0,-1,-1)=1 and (-1,0,-1)=0 and (0,1,-1)=0 and (1,0,-1)=1 } 
rule : {(0,0,0)} 10 { t }

