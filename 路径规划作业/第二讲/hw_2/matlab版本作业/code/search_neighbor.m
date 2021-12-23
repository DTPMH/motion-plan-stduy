function neigh_node=search_neighbor(current_node)
        neigh_node=[current_node(2)-1,current_node(3)+1,14;
                    current_node(2),current_node(3)+1,10;
                    current_node(2)+1,current_node(3)+1,14;
                    current_node(2)-1,current_node(3),10;
                    current_node(2)+1,current_node(3),10;
                    current_node(2)-1,current_node(3)-1,14;
                    current_node(2),current_node(3)-1,10;
                    current_node(2)+1,current_node(3)-1,14;
            ];
end