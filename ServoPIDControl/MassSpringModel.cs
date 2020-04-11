using NLog;

namespace ServoPIDControl
{
    internal class MassSpringModel
    {
        // private static readonly Logger Log = LogManager.GetCurrentClassLogger();

        public double Acceleration { get; private set; }
        public double DamperConstant { get; set; } = 0.1;
        public double Mass { get; set; } = 0.5;
        public double SpringConstant { get; set; } = 10;
        public double Position { get; private set; }
        public double Velocity { get; private set; }

        public void Update(double extForce, double attachPos, double dt)
        {
            var springForce = (attachPos - Position) * SpringConstant;
            var force = springForce + extForce - Velocity * DamperConstant;

            // Log.Info($"extForce: {extForce}, springForce: {springForce}");

            Acceleration = force / Mass;
            Velocity += Acceleration * dt;
            Position += Velocity * dt;

            Position =
                Position < -10 ? -10
                : Position > 10 ? 10
                : Position;
        }
    }
}