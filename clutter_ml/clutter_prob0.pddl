(define (problem prob0)
(:domain clutter)
(:objects
table1 table2 table3 table4 -surface
lgrippertoolframe rgrippertoolframe -manipulator
blue0 green0 blue1 green1 -objs
)

(:init
(on-surface blue0 table2)
(on-surface blue1 table2)
(on-surface green0 table1)
(on-surface green1 table2)
(arm-empty lgrippertoolframe)
(arm-empty rgrippertoolframe)
)

(:goal
(and (on-surface blue0 table3) (on-surface blue1 table3) (on-surface green0 table4) (on-surface green1 table4) (arm-empty lgrippertoolframe) (arm-empty rgrippertoolframe))
)

)
