function quater = Tbn2Q(strapdown)
quater(2,1)=0.5*sqrt(1.0+strapdown(1,1)-strapdown(2,2)-strapdown(3,3));
	   if (strapdown(3,2)-strapdown(2,3))<0
	   quater(2,1)=-quater(2,1); 
       end 
	
       quater(3,1)=0.5*sqrt(1.0-strapdown(1,1)+strapdown(2,2)-strapdown(3,3));
       if (strapdown(1,3)-strapdown(3,1))<0 
       quater(3,1)=-quater(3,1); 
       end
	
       quater(4,1)=0.5*sqrt(1.0-strapdown(1,1)-strapdown(2,2)+strapdown(3,3));
       if (strapdown(2,1)-strapdown(1,2))<0
       quater(4,1)=-quater(4,1); 
       end
	
       quater(1,1)=sqrt(1.0-quater(2,1)^2-quater(3,1)^2-quater(4)^2);%q0È¡Õý 
end