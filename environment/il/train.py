import torch
import torch.nn as nn
import torch.optim as optim
import matplotlib.pyplot as plt
from torch.utils.data import Dataset, DataLoader, TensorDataset
import numpy as np
import h5py
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_absolute_error, mean_squared_error, r2_score

def visualize_results(targets_denorm, predictions_denorm):
    # Create subplots for scatter plots
    plt.figure(figsize=(18, 20))
    
    # Scatter plots of True vs Predicted values
    for i in range(8):
        plt.subplot(4, 2, i+1)
        true = targets_denorm[:, i]
        pred = predictions_denorm[:, i]
        plt.scatter(true, pred, alpha=0.3, label='Samples')
        plt.plot([min(true), max(true)], [min(true), max(true)], 'r--', label='Perfect Prediction')
        plt.xlabel(f'True Value (Thruster {i+1})')
        plt.ylabel(f'Predicted Value (Thruster {i+1})')
        plt.title(f'Thruster {i+1} - True vs Predicted')
        plt.legend()
        plt.grid(True)
        
        # Add R² and MAE to plot
        r2 = r2_score(true, pred)
        mae = mean_absolute_error(true, pred)
        plt.text(0.05, 0.9, f'R²: {r2:.2f}\nMAE: {mae:.2f}', 
                transform=plt.gca().transAxes,
                bbox=dict(facecolor='white', alpha=0.8))
    
    plt.tight_layout()
    plt.savefig('true_vs_predicted_scatter.png')
    plt.show()

    # Create sample comparison plot (first 100 samples)
    plt.figure(figsize=(18, 20))
    sample_indices = np.arange(100)
    
    for i in range(8):
        plt.subplot(4, 2, i+1)
        plt.plot(sample_indices, targets_denorm[:100, i], 'b-', label='True')
        plt.plot(sample_indices, predictions_denorm[:100, i], 'r--', label='Predicted')
        plt.xlabel('Sample Index')
        plt.ylabel('Value')
        plt.title(f'Thruster {i+1} - First 100 Samples')
        plt.legend()
        plt.grid(True)
    
    plt.tight_layout()
    plt.savefig('sample_comparison.png')
    plt.show()

    # Residuals distribution
    plt.figure(figsize=(18, 20))
    residuals = targets_denorm - predictions_denorm
    
    for i in range(8):
        plt.subplot(4, 2, i+1)
        plt.hist(residuals[:, i], bins=50, alpha=0.7)
        plt.xlabel('Residual (True - Predicted)')
        plt.ylabel('Frequency')
        plt.title(f'Thruster {i+1} - Residual Distribution')
        plt.grid(True)
    
    plt.tight_layout()
    plt.savefig('residual_distribution.png')
    plt.show()

# Configuration for feature normalization
class FeatureNormalizer:
    def __init__(self, x_scales, y_scales):
        self.x_scales = np.array(x_scales, dtype=np.float32)
        self.y_scales = np.array(y_scales, dtype=np.float32)
        
    def normalize_x(self, x):
        return x / self.x_scales
    
    def denormalize_x(self, x_norm):
        return x_norm * self.x_scales
    
    def normalize_y(self, y):
        return y / self.y_scales
    
    def denormalize_y(self, y_norm):
        return y_norm * self.y_scales

NU_MIN: float  = 0.0
NU_MAX: float  = 5.0
D_LOC_MAX: float  = 5.0
DEPTH_MIN: float  = 2.0
DEPTH_MAX: float  = 25.0
ZERO: float = 1.0

# Example scaling configuration
X_SCALE_FACTORS = [
    # Current state features
    ZERO, ZERO, DEPTH_MAX,
    np.pi/4, np.pi/4, np.pi,
    NU_MAX, NU_MAX, NU_MAX,
    0.05, 0.05, 0.1,
    # Desired state features
    D_LOC_MAX, D_LOC_MAX, DEPTH_MAX,
    ZERO, ZERO, np.pi,
    NU_MAX, NU_MAX, ZERO,
    ZERO, ZERO, ZERO
]

Y_SCALE_FACTORS = [80.0, 80.0, 80.0, 80.0,  # Main thrusters
                   50.0, 50.0, 50.0, 50.0]   # Tunnel thrusters
normalizer = FeatureNormalizer(X_SCALE_FACTORS, Y_SCALE_FACTORS)

# Load and process data
with h5py.File('mpc_data.h5', 'r') as hf:
    X = np.hstack((hf['x_current'][:], hf['x_desired'][:])).astype(np.float32)
    y = hf['u_opt'][:].astype(np.float32)

# Split dataset into train (60%), val (20%), test (20%)
X_train_val, X_test, y_train_val, y_test = train_test_split(
    X, y, test_size=0.2, shuffle=True, random_state=42)
X_train, X_val, y_train, y_val = train_test_split(
    X_train_val, y_train_val, test_size=0.25, shuffle=True, random_state=42)  # 0.25 * 0.8 = 0.2

# Apply normalization
X_train_norm = normalizer.normalize_x(X_train)
X_val_norm = normalizer.normalize_x(X_val)
X_test_norm = normalizer.normalize_x(X_test)
y_train_norm = normalizer.normalize_y(y_train)
y_val_norm = normalizer.normalize_y(y_val)
y_test_norm = normalizer.normalize_y(y_test)

# Create Tensor datasets
train_dataset = TensorDataset(torch.FloatTensor(X_train_norm), torch.FloatTensor(y_train_norm))
val_dataset = TensorDataset(torch.FloatTensor(X_val_norm), torch.FloatTensor(y_val_norm))
test_dataset = TensorDataset(torch.FloatTensor(X_test_norm), torch.FloatTensor(y_test_norm))

# Network Architecture (unchanged)
class FossenNet(nn.Module):
    def __init__(self):
        super(FossenNet, self).__init__()
        self.net = nn.Sequential(
            nn.Linear(24, 512),
            nn.BatchNorm1d(512),
            nn.LeakyReLU(0.1),
            nn.Dropout(0.3),
            
            nn.Linear(512, 256),
            nn.BatchNorm1d(256),
            nn.LeakyReLU(0.1),
            nn.Dropout(0.2),
            
            nn.Linear(256, 128),
            nn.BatchNorm1d(128),
            nn.LeakyReLU(0.1),
            
            nn.Linear(128, 64),
            nn.LeakyReLU(0.1),
            
            nn.Linear(64, 8)
        )
        
        for layer in self.net:
            if isinstance(layer, nn.Linear):
                nn.init.kaiming_normal_(layer.weight, nonlinearity='leaky_relu')
                nn.init.constant_(layer.bias, 0.1)

    def forward(self, x):
        return self.net(x)

# Training configuration
config = {
    'batch_size': 256,
    'lr': 3e-4,
    'epochs': 5,
    'weight_decay': 1e-6,
    'patience': 15
}

def train():
    model = FossenNet()
    criterion = nn.MSELoss()
    optimizer = optim.AdamW(model.parameters(), lr=config['lr'], weight_decay=config['weight_decay'])
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, 'min', patience=7, factor=0.5)
    
    train_loader = DataLoader(train_dataset, batch_size=config['batch_size'], shuffle=True, num_workers=2)
    val_loader = DataLoader(val_dataset, batch_size=config['batch_size'], num_workers=2)
    
    best_loss = float('inf')
    no_improve = 0
    
    for epoch in range(config['epochs']):
        # Training phase
        model.train()
        train_loss = 0
        for x_batch, y_batch in train_loader:
            optimizer.zero_grad()
            pred = model(x_batch)
            loss = criterion(pred, y_batch)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            optimizer.step()
            train_loss += loss.item()
        
        # Validation phase
        model.eval()
        val_loss = 0
        with torch.no_grad():
            for x_val, y_val in val_loader:
                pred = model(x_val)
                val_loss += criterion(pred, y_val).item()
                
        avg_train = train_loss/len(train_loader)
        avg_val = val_loss/len(val_loader)
        scheduler.step(avg_val)
        
        # Early stopping check
        if avg_val < best_loss:
            best_loss = avg_val
            no_improve = 0
            torch.save(model.state_dict(), 'best_model.pth')
        else:
            no_improve += 1
            
        if no_improve >= config['patience']:
            print(f"Early stopping at epoch {epoch+1}")
            break
            
        print(f"Epoch {epoch+1}/{config['epochs']} | "
              f"Train: {avg_train:.4f} | Val: {avg_val:.4f} | "
              f"LR: {optimizer.param_groups[0]['lr']:.2e}")

def evaluate_test_set():
    model = FossenNet()
    model.load_state_dict(torch.load('best_model.pth'))
    model.eval()
    
    test_loader = DataLoader(test_dataset, batch_size=config['batch_size'], shuffle=False, num_workers=2)
    
    predictions = []
    targets = []
    with torch.no_grad():
        for x_batch, y_batch in test_loader:
            pred = model(x_batch)
            predictions.append(pred.numpy())
            targets.append(y_batch.numpy())
    
    predictions = np.vstack(predictions)
    targets = np.vstack(targets)
    
    # Denormalize
    predictions_denorm = normalizer.denormalize_y(predictions)
    targets_denorm = normalizer.denormalize_y(targets)
    
    # Calculate metrics
    metrics = {
        'MAE': mean_absolute_error(targets_denorm, predictions_denorm),
        'MSE': mean_squared_error(targets_denorm, predictions_denorm),
        'RMSE': np.sqrt(mean_squared_error(targets_denorm, predictions_denorm)),
        'R2': r2_score(targets_denorm, predictions_denorm)
    }
    
    # Per-thruster metrics
    thruster_metrics = []
    for i in range(8):
        thruster_metrics.append({
            'Thruster': i+1,
            'MAE': mean_absolute_error(targets_denorm[:, i], predictions_denorm[:, i]),
            'MSE': mean_squared_error(targets_denorm[:, i], predictions_denorm[:, i]),
            'RMSE': np.sqrt(mean_squared_error(targets_denorm[:, i], predictions_denorm[:, i])),
            'R2': r2_score(targets_denorm[:, i], predictions_denorm[:, i])
        })
    
    return metrics, thruster_metrics, predictions_denorm, targets_denorm

if __name__ == '__main__':
    train()
    
    # Evaluate on test set
    metrics, thruster_metrics, preds, targets = evaluate_test_set()
    
    print("\nFinal Test Set Metrics:")
    print(f"MAE: {metrics['MAE']:.4f}")
    print(f"MSE: {metrics['MSE']:.4f}")
    print(f"RMSE: {metrics['RMSE']:.4f}")
    print(f"R²: {metrics['R2']:.4f}")
    
    print("\nPer-Thruster Metrics:")
    for tm in thruster_metrics:
        print(f"\nThruster {tm['Thruster']}:")
        print(f"MAE: {tm['MAE']:.4f}  MSE: {tm['MSE']:.4f}")
        print(f"RMSE: {tm['RMSE']:.4f}  R²: {tm['R2']:.4f}")
        
    visualize_results(targets, preds)