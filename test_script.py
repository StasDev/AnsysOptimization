"""
Order to set the calculation begin:
1.Run start_calculation()

How start_calculation actually works:
1.From test_script.py run update_acp_pre
2.update_acp_pre call acp_pre.py
3.From test_script.py run update_project
4.From test_script.py run update_acp_post
5.update_acp_post call acp_post.py
"""

from random import randint
import my_orm

def logging(message):
    """This function create the new records in logging file"""
    f = open(r'C:\Users\1\Desktop\Work\Lopast_helicopter_13_10\Scripts\log.txt', 'a')
    f.write(message + '\n')
    f.close()


def update_acp_pre():
    """This function update the ACP pre window by running the acp_pre.py"""
    try:
        system1 = GetSystem(Name="ACP-Pre")
        setup1 = system1.GetContainer(ComponentName="Setup")
        setup1.RunScript(ScriptPath=r"C:\Users\1\Desktop\Work\Lopast_helicopter_13_10\Scripts\acp_pre.py")
    except:
        logging('Update ACP failed')
    else:
        logging('Update ACP success')


def update_project():
    """This function update the project in Workbench window"""
    try:
        system1 = GetSystem(Name="ACP-Pre")
        setup1 = system1.GetContainer(ComponentName="Setup")
        Parameters.SetDesignPointsOutOfDate()
        engineeringData1 = system1.GetContainer(ComponentName="Engineering Data")
        geometry1 = system1.GetContainer(ComponentName="Geometry")
        model1 = system1.GetContainer(ComponentName="Model")
        system2 = GetSystem(Name="ACP-Post")
        results1 = system2.GetContainer(ComponentName="Results")
        system3 = GetSystem(Name="CFX")
        mesh1 = system3.GetContainer(ComponentName="Mesh")
        setup2 = system3.GetContainer(ComponentName="Setup")
        solution1 = system3.GetContainer(ComponentName="Solution")
        results2 = system3.GetContainer(ComponentName="Results")
        system4 = GetSystem(Name="SYS")
        model2 = system4.GetContainer(ComponentName="Model")
        setup3 = system4.GetContainer(ComponentName="Setup")
        solution2 = system4.GetContainer(ComponentName="Solution")
        results3 = system4.GetContainer(ComponentName="Results")
        Parameters.SetRetainedDesignPointDataInvalid(InvalidContainers=[engineeringData1, geometry1, model1, results1, setup1, mesh1, setup2, solution1, results2, model2, setup3, solution2, results3])
        setupComponent1 = system1.GetComponent(Name="Setup")
        modelComponent1 = system4.GetComponent(Name="Model")
        setupComponent2 = system4.GetComponent(Name="Setup")
        solutionComponent1 = system4.GetComponent(Name="Solution")
        resultsComponent1 = system4.GetComponent(Name="Results")
        resultsComponent2 = system2.GetComponent(Name="Results")
        MarkComponentsOutOfDateForDps(Components=[setupComponent1, modelComponent1, setupComponent2, solutionComponent1, resultsComponent1, resultsComponent2])
        Parameters.SetPartiallyRetainedDataInvalid(Containers=[setup1, model2, setup3, solution2, results3, results1])
        Update()
    except:
        logging('Update_failing')
    else:
        logging('Update success')


def update_acp_post():
    """This function get the values from ACP Post window by running the acp_post.py"""
    try:
        system1 = GetSystem(Name="ACP-Post")
        setup1 = system1.GetContainer(ComponentName="Results")
        setup1.RunScript(ScriptPath=r"C:\Users\1\Desktop\Work\Lopast_helicopter_13_10\Scripts\acp_post.py")
    except:
        logging('Error when trying get the values from ACP')
    else:
        logging('Get the values from ACP success')
    

def put_values_into_alhoritm():
    """This function put the new values to the optimization alhoritm"""
    f = open(r'C:\Users\1\Desktop\Work\Lopast_helicopter_13_10\Scripts\angles.txt', 'w')
    for i in range(randint(5, 20)):
        value = randint(0, 900) / 10
        f.write(str(value) + '\n')
    f.close()


update_acp_pre()
update_project()
update_acp_post()