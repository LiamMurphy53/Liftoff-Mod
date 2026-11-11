using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using UnityEngine;

namespace GoProGeometry
{
    // Liftoff's body drag is 1/2 · 1.184 · Cd · A(direction) · v², opposite the drone's velocity through the air.
    // Cd is the frame's DragCoefficient. A comes from a table Liftoff fills when it builds the drone, by measuring
    // the drone's silhouette from a fixed set of directions in the drone's frame, and is blended between the
    // nearest entries in flight. Writing CdA / Cd into that table gives the profile's drag through Liftoff's own
    // drag code, with its wind and blending untouched. Prop drag (a separate Liftoff term) is left as it is.
    internal sealed class DragAdapter
    {
        static readonly Module Game=typeof(FlightManager).Module;
        static readonly FieldInfo Owner=Game.ResolveField(GameTokens.PropulsionOwner);
        static readonly MethodInfo Holder=(MethodInfo)Game.ResolveMethod(GameTokens.DragHolder), Areas=(MethodInfo)Game.ResolveMethod(GameTokens.DragAreas),
            SetAreas=(MethodInfo)Game.ResolveMethod(GameTokens.DragAreasSetter), DirectionVector=(MethodInfo)Game.ResolveMethod(GameTokens.DragDirectionVector);

        readonly object holder;
        readonly IDictionary stock, adapted;
        internal string Report { get; }

        // Liftoff's current table as (direction of travel in profile axes, Cd·A in m²).
        internal static (float[] directions,float[] areaM2) Read(Component controller,float cd)
        {
            var table=Table(HolderOf(controller));
            var dirs=new List<float>(); var areas=new List<float>();
            foreach(DictionaryEntry e in table)
            {
                var u=(Vector3)DirectionVector.Invoke(e.Key,null);
                dirs.AddRange(GeometryMath.ProfileVector(u.x,u.y,u.z).Select(v=>(float)v));
                areas.Add((float)e.Value*cd);
            }
            return (dirs.ToArray(),areas.ToArray());
        }

        internal DragAdapter(Component controller,GeometryProfile profile,float cd)
        {
            if(!(cd>0.01f && cd<20f)) throw new InvalidOperationException("This frame's drag coefficient ("+cd+") can't carry a drag table.");
            var target=DragTable.FromProfile(profile);
            holder=HolderOf(controller);
            stock=Table(holder);
            adapted=(IDictionary)Activator.CreateInstance(stock.GetType());
            if(!SetAreas.GetParameters()[0].ParameterType.IsInstanceOfType(adapted)) throw new InvalidOperationException("Liftoff's drag table changed type.");
            foreach(DictionaryEntry e in stock)
            {
                var u=(Vector3)DirectionVector.Invoke(e.Key,null);
                var p=GeometryMath.ProfileVector(u.x,u.y,u.z);
                adapted[e.Key]=(float)(target.Area(p[0],p[1],p[2])/cd);
            }
            var read=Read(controller,cd); var before=new DragTable(read.directions,read.areaM2);
            string Summary(DragTable t)=>Cm2(t.Area(0,-1,0))+" / "+Cm2(t.Area(0,0,1))+" / "+Cm2(t.Area(1,0,0));
            Report="drag table, Cd·A forward/up/sideways "+Summary(target)+" cm² (Liftoff "+Summary(before)+")";
        }
        static string Cm2(double m2)=>(m2*1e4).ToString("F0");

        internal void Install() { SetAreas.Invoke(holder,new object[]{adapted}); Verify(); }
        internal void Verify()
        {
            if(!ReferenceEquals(Areas.Invoke(holder,null),adapted)) throw new InvalidOperationException("Liftoff rebuilt its drag table. Stop and reapply the profile.");
        }
        internal void Restore() { if(ReferenceEquals(Areas.Invoke(holder,null),adapted)) SetAreas.Invoke(holder,new object[]{stock}); }

        static object HolderOf(Component controller)
        {
            var owner=Owner.GetValue(controller) ?? throw new InvalidOperationException("The drone has no propulsion model yet.");
            return Holder.Invoke(owner,null) ?? throw new InvalidOperationException("Liftoff hasn't measured this drone's drag yet.");
        }
        static IDictionary Table(object holder)
        {
            var table=Areas.Invoke(holder,null) as IDictionary ?? throw new InvalidOperationException("Liftoff's drag table has an unexpected type.");
            if(table.Count<6) throw new InvalidOperationException("Liftoff's drag table has only "+table.Count+" directions.");
            return table;
        }
    }
}
