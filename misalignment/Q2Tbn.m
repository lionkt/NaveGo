function strapdown = Q2Tbn(quater)
strapdown = [quater(1)^2+quater(2)^2-quater(3)^2-quater(4)^2,2*(quater(2)*quater(3)-quater(1)*quater(4)),2*(quater(2)*quater(4)+quater(1)*quater(3));
              2*(quater(2)*quater(3)+quater(1)*quater(4)),quater(1)^2-quater(2)^2+quater(3)^2-quater(4)^2,2*(quater(4)*quater(3)-quater(1)*quater(2));
                2*(quater(2)*quater(4)-quater(1)*quater(3)),2*(quater(4)*quater(3)+quater(1)*quater(2)),quater(1)^2-quater(2)^2-quater(3)^2+quater(4)^2;];
end