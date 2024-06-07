# SOFA Tearing Plugin

Tearing plugin is a simulation tool designed to replicate the tearing
process, offering users the flexibility to control and observe fracture
scenarios. Whether you prefer specifying a fracture scenario with
precision or allowing fractures to occur dynamically due to simulated
forces, this plugin empowers you with versatile tearing simulations.

### Features

-   Low-level API (C++ methods) is accessible through the
    **TearingAlgorithms** objects for surface tissue, enabling the
    computation of the fracture path from a specified point in a
    particular direction. Similarly, **VolumeTearingAlgorithms**
    facilitates the computation of plane intersections from elongated
    tetrahedra.

-   A high-level API is also accessible through SOFA components, namely
    the **TearingEngine** and **VolumeTearingEngine**. These components
    can be employed in a SOFA scene to execute the fracture operation,
    allowing for either the specification of a starting point and
    fracture direction or a force-driven fracture.

-   Several data elements are available to control the fracture, such
    as:

    -   stressThreshold: Threshold value for element stress which will
        determine candidate elements (triangles) to initiate the
        fracture.

    -   fractureMaxLength: Maximum length of the fracture segment per
        simulation step.

    -   nbFractureMax: Maximum number of allowed fracture occurrences
        for the simulation.

## Installation

This plugin should be added as an external plugin of SOFA using the
CMAKE_EXTERNAL_DIRECTORIES CMake variable of SOFA. See SOFA
documentation for more information.

## Getting Started with Tearing Plugin

### Flexible Fracture Scenarios: TearingScenarioEngine
Using TearingScenarioEngine, you can leverage its capabilities for benchmarking and illustration purposes. This mode provides a structured environment where predefined scenarios can be executed, allowing for accurate performance measurements and visualization of results.

Several scenarios through SOFA scene files are available in *scenes*
directory. Corresponding screenshots of the results can be found in the
*doc* directory.

To use TearingScenarioEngine, you need to provide the following details:

1.  Staring Point Index(startVertexId): The initial vertex where the fracture will initiate.
2. Starting Triangle Index(startTriangleId): The triangle index in which the starting vertex is chosen.
3.  Fracture Direction(startDirection): The direction in which the fracture will propagate.
4.  Fracture Length(startLength): The length of the fracture to control its extent.
   

<details>
<summary>Example</summary>

    <TearingScenarioEngine name="TearingScenarioEngine" input_position="@Mo.position" startVertexId = "14" startDirection="1.0 0.0 0.0" startTriangleId="13" startLength="5"/>
</details>

### Force-Driven Tearing: TearingEngine

Tearing plugin provides the capability to simulate tearing dynamically,
allowing fractures to occur naturally in response to applied forces
without the need to explicitly specify a fracture scenario. This feature
enables users to observe the real-time effects of forces on the
simulated material.

<details>
<summary>Example</summary>

    <ConstantForceField name="CFF" indices="462" forces="0 -300 0" showArrowSize=".01" />
    <TriangularFEMForceField name="FEM" youngModulus="100" poissonRatio="0.3" method="large"/>
    <TearingEngine name="TearingEngine" input_position="@Mo.position" stressThreshold="0.1" step="20" nbFractureMax="4" />

</details>


**Remark.** it\'s important to note that the simulation\'s behaviour is
contingent upon both the properties of the applied forces, such as
magnitude, and the specified material property, particularly the stress threshold. For example, changing the data parameters for a same mesh cause different results:

<div style="text-align: center;">
        <figure><img   src="https://raw.githubusercontent.com/InfinyTech3D/Tearing/9b12adcbbf5830dd85942d1d998587a242de3dbc/doc/Images/Picture1.jpg" alt="Alt Text">  <figcaption>  </figcaption>
          </figure>
    </div>

<details>
<summary></summary>
   
    <TearingEngine name="TearingEngine" input_position="@Mo.position"stressThreshold="5.0" step="20" nbFractureMax="4" />
 </details>
<figure><img src="https://raw.githubusercontent.com/InfinyTech3D/Tearing/9b12adcbbf5830dd85942d1d998587a242de3dbc/doc/Images/Picture2.jpg" alt="Alt Text">  <figcaption>  </figcaption>  </figure>
<details>
<summary></summary>
   
    \<TearingEngine name="TearingEngine" input_position="@Mo.position"stressThreshold="1.0" step="20" nbFractureMax="4"/>
 </details>




#### Principal Steps Of Force-Driven Fracture Simulation

Our fracture simulation follows four principal steps:

1.  It generates a list of candidate triangles capable of experiencing
    fractures. This list is determined based on stressThreshold. Each
    triangle with a principal stress value exceeding this specified
    threshold is included in the candidate list.

2.  Among all the triangles included in the list, the one with the
    highest value is considered as the triangle to perform the fracture.

3.  To determine the starting vertex, there are three available options. Let $\sigma_{i}$ indicates the  principal stress values of triangles adjacent to vertex $j$ in the candidate triangle. In each method, the vertex with maximum $s_{j}$ value is selected as the starting point for fracture, see [Figure 1](#figure-1).
<div style="text-align: center;">
      <figure id="figure-1">
           <img      src="https://raw.githubusercontent.com/InfinyTech3D/Tearing/165004ac22a47a508078561376e9003c6067fd20/doc/Images/Picture3.png" alt="Starting Vertex" style="width:50px; height:auto;">
         <figcaption>
    Figure 1: A value is attributed to vertex j based on the principal values of the triangles adjacent to it.
         </figcaption>
      </figure>
</div>

4.  To initiate the fracture, it is necessary to determine both its
    direction and length. 
    The direction of the fracture is calculated to be perpendicular to the first dominant principal stress direction, and the fracture length is specified by the user. You can define this parameter by passing value to fractureMaxLength.

<figure id="figure-2">
    <img src="https://raw.githubusercontent.com/InfinyTech3D/Tearing/9b12adcbbf5830dd85942d1d998587a242de3dbc/doc/Images/Picture4.png" alt="Tearing Plugin" style="width:100%">
    <figcaption>Figure 2: A global overview of the interconnected components and their
sequential interactions.</figcaption>
</figure>


#### Methods To Choose The Starting Vertex
1. Using area-weighted average, we define $s_{j}$:
$$
s_{j} = \frac{\sum_{i}^{}{\left| \sigma_{T_{i}} \right|A_{T_{i}}}}{\sum_{i}^{}A_{T_{i}}}
$$
, where $\ A_{T_{i}}$ indicates the area of the $i$-th triangle adjacent to vertex $j$. To activate this option, you need to pass WeightedAverageArea to ComputeVertexStressMethod.

<details>
<summary>Example</summary>

    \<TearingEngine name="TearingEngine" input_position="@Mo.position" ComputeVertexStressMethod= "WeightedAverageArea" />

</details>

2. Using unweighted-average, we define $s_{j}$:
$$
s_{j} = \sum_{}^{}\left| \sigma_{T_{i}} \right|.
$$

To activate this option, you need to pass UnweightedAverageArea to
ComputeVertexStressMethod.

<details>
<summary>Example</summary>

    <TearingEngine name="TearingEngine" input_position="@Mo.position" ComputeVertexStressMethod= "UnweightedAverageArea" />
</details>

3. Using reciprocal-distance weighted average,
$$
s_{j} = \sum_{i}^{}\frac{\left| \sigma_{T_{i}} \right|}{d_{ij}}.
$$

To activate this option, you need to pass WeightedAverageInverseDistance
to ComputeVertexStressMethod.

<details>
<summary>Example</summary>

    <TearingEngine name="TearingEngine" input_position="@Mo.position" ComputeVertexStressMethod= "WeightedAverageInverseDistance" />
</details>



## Technical Look Inside The Tearing Plugin

This section provides a comprehensive overview of the seamless
integrated functions designed to simulate tearing.
<details>
<summary>Function</summary>

    void BaseTearingEngine<DataTypes>::triangleOverThresholdPrincipalStress();
</details>

**Task.**
-   A list of triangles is generated based on the first dominant
    principal stress value of each triangle. This list stores the index
    of triangles with stress values exceeding the specified threshold.

-   From the triangles in this list, the one with the highest principal
    stress value is selected as the potential candidate for fracture
    initiation.

-   Among the vertices of the selected triangle, the starting point for
    the fracture is determined based on the chosen option for
    ComputeVertexStressMethod.
<details>
<summary>Function</summary>

    void BaseTearingEngine<DataTypes>::updateTriangleInformation();
</details>

**Task.** 

For each triangle in the mesh, essential information--- including *stress (*$stress\  = \ K*strain$*)*, *maxStress* (the first dominant principal stress value), and *principalStressDirection* (the corresponding principal stress
direction) --- is initialized.

<details>
<summary>Function</summary>

    void BaseTearingEngine<DataTypes>::doUpdate();
</details>

**Task.**
-   To enhance simulation performance, certain triangles are excluded
    from the candidate selection process, and, consequently, they do not
    contribute to the simulation. Additionally, during the simulation,
    two specific cases, namely X-junction and T-junction, may arise
    based on the potential triangle and its fracture direction. In these
    instances, distinct actions are taken. This filtering process is
    enabled by default.

-   The information for all eligible triangles is initialized by updateTriangleInformation.

-   The potential triangle is selected by  triangleOverThresholdPrincipalStress.

<details>
<summary>Function</summary>

    void BaseTearingEngine<DataTypes>::computeEndPoints();
</details>

**Task.** 

Using the fracture direction and the user-defined maximum
length, the coordinates of the two endpoints of the fracture segment are
calculated.
<details>
<summary>Function</summary>

    void TearingAlgorithms<DataTypes>::algoFracturePath (...);
</details>

**Task.**

-   Given the starting point, the function computes the intersection of
    the fracture segment with the mesh.

-   If any intersection exists:

    -   All intersection objects are converted to meaningful mesh
        elements by
        TearingAlgorithms\<DataTypes\>::pathAdaptationObject.

    -   Utilizing the topological information from the previous step, a
        remeshing operation is executed using
        TearingAlgorithms\<DataTypes\>::splitting.

    <!-- -->

    -   If no intersection is found, two special cases are checked:
        X-junction and T-junction.

        -   In the case of an X-junction, specific action is performed
            to proceed with the fracture.

        -   For a T-junction, the index of this potential triangle is
            added to the list of triangles to be ignored.

<details>
<summary>Function</summary>

    void TearingEngine<DataType>::algoFracturePath();
</details>

**Task.**

-   The fracture segment is calculated by computeEndPoints.

-   The fracture is performed by
    TearingAlgorithms\<DataTypes\>::algoFracturePath.
