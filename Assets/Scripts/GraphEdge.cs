using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GraphEdge
{
    public Node start_node;
    public Node end_node;
    public float cost;

    public GraphEdge(Node start_node, Node end_node, float cost)
    {
        this.start_node = start_node;
        this.end_node = end_node;
        this.cost = cost;
    }
    public GraphEdge(Node start_node, Node end_node)
    {
        this.start_node = start_node;
        this.end_node = end_node;
        this.cost = 0;
    }
    // override object.Equals
    public override bool Equals(object obj)
    {
        //
        // See the full list of guidelines at
        //   http://go.microsoft.com/fwlink/?LinkID=85237
        // and also the guidance for operator== at
        //   http://go.microsoft.com/fwlink/?LinkId=85238
        //

        if (obj == null || GetType() != obj.GetType())
        {
            return false;
        }

        return start_node.Equals(((GraphEdge)obj).start_node) && end_node.Equals(((GraphEdge)obj).end_node) || start_node.Equals(((GraphEdge)obj).end_node) && end_node.Equals(((GraphEdge)obj).start_node);
    }

    // override object.GetHashCode
    public override int GetHashCode()
    {
        return base.GetHashCode();
    }
}