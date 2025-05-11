package vanettutorials.simulations.circle;
import org.car2x.veins.nodes.Scenario;
import org.car2x.veins.nodes.RSU;

network myTestNetwork extends Scenario
{
    submodules:
        rsu[1]: RSU {
            @display("p=150,140;i=veins/sign/yellowdiamond;is=vs");
        }
}